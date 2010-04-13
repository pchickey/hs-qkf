-- Test of Quaternion Kalman Filter
-- Pat Hickey 19 Mar 10

{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables #-}
module QkfTest where
import Control.Monad
import Control.Concurrent
import System.Random
import Numeric.LinearAlgebra.Static
import Graphics.Gnuplot.Simple
import Qkf

-- All Euler angles use the 313 convention. I realize the aerospace convention is 123 but I goofed;
-- for testing purposes we can continue to use 313
-- see "Representing Attitude: Euler Angles, Unit QUaternions, and Rotation Vectors" James Diebel 2006
-- for reference on all conversions.
type Angle = Double
type RotMatrix = Matrix (D3,D3) Double
type Eulers = Vector D3 Double 
type WorldAngularRate = Vector D3 Double
-- ordering is [phi, theta psi] naturally
phi :: Eulers -> Angle
phi e = e @> 0
theta :: Eulers -> Angle
theta e = e @> 1
psi :: Eulers -> Angle
psi e = e @> 2
-- For convenience:
ezero :: Eulers
ezero = [$vec|0,0,0|]

-- Make a rotation matrix from Euler angles. Deibel (49)
matOfEulers :: Eulers -> RotMatrix
matOfEulers eulers = [$mat| cphi*cpsi - sphi*cthe*spsi,  cphi*spsi + sphi*cthe*cpsi, sphi*sthe;
                           -sphi*cpsi - cphi*cthe*spsi, -sphi*spsi + cphi*cthe*cpsi, cphi*sthe;
                            sthe*spsi,                   sthe*cpsi,                  cthe      |]
  where 
    cphi = cos . phi   $ eulers 
    sphi = sin . phi   $ eulers 
    cthe = cos . theta $ eulers
    sthe = sin . theta $ eulers 
    cpsi = cos . psi   $ eulers
    spsi = sin . psi   $ eulers

-- Quaternion convention:
-- In the James Deibel paper, quaternions are [re i j k] ordered; 
-- type Quat in module Qkf is [i j k re] ordered.
-- Therefore, use these accessors for the component within this program
q0 :: Quat -> Double
q0 = (@>3) 
q1 :: Quat -> Double
q1 = (@>0)
q2 :: Quat -> Double
q2 = (@>1) 
q3 :: Quat -> Double
q3 = (@>2) 

-- Make a quaternion from Euler angles. Deibel (66), except re-ordered with real component last
-- to fit type Quat convention.
qOfEulers :: Eulers -> Quat
qOfEulers eulers = 
  [$vec| cphi * cpsi * sthe + sphi * sthe * spsi,
         cphi * sthe * spsi - sphi * cpsi * sthe,
         cphi * cthe * spsi + cthe * cpsi * sphi,
         cphi * cthe * cpsi - sphi * cthe * spsi|]
  where 
    cphi = cos . (/2) . phi   $ eulers 
    sphi = sin . (/2) . phi   $ eulers 
    cthe = cos . (/2) . theta $ eulers
    sthe = sin . (/2) . theta $ eulers 
    cpsi = cos . (/2) . psi   $ eulers
    spsi = sin . (/2) . psi   $ eulers
-- Deibel (55)
eulersOfQ :: Quat -> Eulers
eulersOfQ q =
  [$vec| atan2 (2*qq1*qq3 - 2*qq0*qq2) (2*qq2*qq3 + 2*qq0*qq1)
       ,  acos (qq3*qq3 - qq2*qq2 - qq1*qq1 + qq0*qq0)
       ,  atan2 (2*qq1*qq3 + 2*qq0*qq2) (2*qq0*qq1 - 2*qq2*qq3) |]
  where qq0 = q0 q; qq1 = q1 q; qq2 = q2 q; qq3 = q3 q

-- in (1,2,3) sequence: Deibel (73)
eulers123OfQ q = 
  [$vec| atan2 (2*qq2*qq3 + 2*qq0*qq1) (qq3*qq3 - qq2*qq2 - qq1*qq1 + qq0*qq0)
       , (-1)*asin (2*qq1*qq3 - 2*qq0*qq2)
       , atan2 (2*qq1*qq2 + 2*qq0*qq3) (qq1*qq1 + qq0*qq0 - qq3*qq3 - qq2*qq2) |]
  where qq0 = q0 q; qq1 = q1 q; qq2 = q2 q; qq3 = q3 q

-- in (3,2,1) sequence: Deibel 
eulers321OfQ q = 
  [$vec| atan2 ((-2)*qq1*qq2 + 2*qq0*qq3) (qq1*qq1 + qq0*qq0 - qq3*qq3 - qq2*qq2)
       , asin (2*qq1*qq3 + 2*qq0*qq2)
       , atan2 ((-2)*qq2*qq3 + 2*qq0*qq1) (qq3*qq3 - qq2*qq2 - qq1*qq1 + qq0*qq0) |]
  where qq0 = q0 q; qq1 = q1 q; qq2 = q2 q; qq3 = q3 q

-- Euler Angle Rate Matrix
-- omega = E(u) * udot  where omega is body fixed angular rates, u is euler angles
-- Defined in Deibel (37-39)
type Emat = Matrix (D3,D3) Double
-- Deibel (56)
ematOfEulers :: Eulers -> Emat
ematOfEulers e = [$mat| sthe*spsi, cpsi, 0;
                       -sthe*cpsi, spsi, 0;
                        cthe,      0,    1|]
  where
    cthe = cos . theta $ e
    sthe = sin . theta $ e
    cpsi = cos . psi   $ e
    spsi = sin . psi   $ e

-- Measurment functions: generate a filter input from Euler angles

toVecMeasurment :: MeasurmentSource -> RefVec -> Eulers -> Measurment
toVecMeasurment s r e = 
  Measurment{ source = s
            , body = (matOfEulers e) <> r
            , ref = r
            , meascov = liftMatrix (* constant 0.00001) (atRows ident d3) }

toAccMeasurment = toVecMeasurment Accelerometer [$vec|0,0,-1|] 
toMagMeasurment = toVecMeasurment Magnetometer [$vec|1,0,0|] 

toGyroMeasurment :: Eulers -> WorldAngularRate-> Measurment
toGyroMeasurment e edot = 
  Measurment { source = Gyro
             , body = (ematOfEulers e) <> edot 
             , ref = [$vec|0,0,0|]
             , meascov = liftMatrix (* constant 0.01) (atRows ident d3) }

-- given: initial euler angles, list of derivatives, time step between derivatives
-- produces: list of resulting euler angles
generateWalk :: Eulers -> [WorldAngularRate]-> Time -> [Eulers]
generateWalk e (edot:edots) dt = (e:es)
                  where e' = advanceConst e edot dt
                        es = generateWalk e' edots dt
generateWalk _ [] _ = []

-- I believe that, since the world frame Eulers are independent variables,
-- I can do a rectangular integration here safely.
advanceConst :: Eulers -> WorldAngularRate -> Time -> Eulers
advanceConst angle angledot dt = angle + (angledot * constant dt)

feedfilter :: Time -> [(Measurment, Measurment, Measurment)] -> (FilterState, RateEstimate) -> [(FilterState, RateEstimate)]
feedfilter dt ((ma, mm, mg):ms) (state,rate) = ((state,rate):states)
  where 
    s'acc = measurmentUpdate ma state
    s'mag = measurmentUpdate mm s'acc
    rate' = rateEstimateUpdate mg dt rate
    s'gyro = timePropogate rate' s'mag
    states = feedfilter dt ms (s'gyro, rate')

statenorm = sumsq . q  
          where sumsq qq = (qq@>0)*(qq@>0) + (qq@>1)*(qq@>1) + (qq@>2)*(qq@>2) + (qq@>3)*(qq@>3)

-- The following functions demonstrate my ignorance of Haskell:
qaccs = [ q0, q1, q2, q3 ]
stateqs fs = map (\qacc -> map (qacc . q . fst) fs) qaccs
justqs  qs = map (\qacc -> map qacc qs) qaccs
angleaccs = [phi, theta, psi]
stateangles  as = map (\angleacc -> map (angleacc . eulersOfQ . q  . fst) as) angleaccs 
angles as = map (\angleacc -> map angleacc as) angleaccs

meas :: [Eulers] -> [WorldAngularRate] -> [(Measurment, Measurment, Measurment)]
meas es edots = zipWith (\e edot -> 
                      (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e edot))
                    es edots
 
statictest = do
  let e = [$vec| pi/4, pi/2, 0 |] :: Eulers
  let edot = [$vec| 0,0,0 |]
  let f = feedfilter 0.05 (repeat 
                      (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e edot)) 
                     (fszero, rezero)
  plotLists [] $ stateqs $ take 10 f
  plotLists [] $ justqs $ take 10 $ repeat $ qOfEulers e
  let q10th =  q . fst . last . take 10 $ f
  putStrLn "Filtered quaternion after 10 iterations:"
  putStrLn $ show q10th
  putStrLn "Theoretical static quaternion:"
  putStrLn $ show $ qOfEulers e
  putStrLn "Supplied set of angles"
  putStrLn $ show e
  putStrLn "Set of angles derived from filtered quaternion"
  putStrLn $ show $ eulersOfQ q10th

velocitytest = do
  let einit = [$vec| pi/4, pi/2, 0 |] :: Eulers
  let edot = [$vec|pi/4, -pi/12, 0|] :: WorldAngularRate
  let edots = (replicate 25 edot) ++ repeat [$vec|0,0,0|]
  let dt = 0.05 -- 20hz
  let walk = generateWalk einit edots dt
 
  let f = feedfilter dt (meas walk edots) (fszero, rezero) 
  let fs = take 50 f
  let ws = take 50 walk

  plotLists [] $ stateangles fs ++ angles ws
  plotLists [] $ stateqs fs ++ (justqs $ map qOfEulers ws)
  return f

iotest :: [a] -> SampleVar a -> IO ()
iotest fs var = 
  forM_ fs (\e -> writeSampleVar var e >>
                  threadDelay 50000 )


-- rosetta code provided a gaussian snippet, somewhat refactored here
-- weaves RandomGen in and out.
gauss :: (RandomGen g) => Double -> g -> (Double, g)
gauss sigma g = ( sigma * sqrt (-2 * log r1) * cos ( 2 * pi * r2 ) , g2 ) 
  where 
    (r1, g1) = random g
    (r2, g2) = random g1

{- needs rewriting. 
advanceRandAngle :: (RandomGen g) => Double -> Angle -> Time -> g -> (Angle, g)
advanceRandAngle sm ang dt g = ( Angle { a = a', da = da' }, g')
  where 
    (gauss1, g') = gauss (30*pi/180) g
    da' = sm * (da ang) + (1-sm) * gauss1 
    a' = (a ang) + da' * dt
                            
advanceEulerR :: (RandomGen g) => Eulers -> g -> (Eulers, g)
advanceEulerR e g = ( Eulers { alpha = alpha', beta = beta', gamma = (gamma e) } , g2 )
  where 
    (alpha', g1) = advanceRandAngle smoothness (alpha e) 0.02 g
    (beta', g2) = advanceRandAngle smoothness (beta  e) 0.02 g1
    smoothness = 0.95

generateRWalk :: (RandomGen g) => Eulers -> g -> [Eulers]
generateRWalk x g = (v:vs)
  where 
    (v, g') = advanceEulerR x g
    vs = generateRWalk v g'
-}

----------------------------------------------------------------------
{-
rtest = do
  g <- getStdGen
  let walk = generateRWalk ezero g
  let walkpairs = map (\e -> ( (a $ alpha e), (a $ beta e) )) walk
  --plotPath [] $ take 100 walkpairs
  let meas = map (\e -> (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e)) 
  let f = feedfilter (meas walk) (fszero, rezero)
  
  --plotLists [] $ stateqs $ take 100 f
{-
  -- fangles isn't producing anything that makes much sense.
  let fangles = map (qtoEtuple . q . fst) f
  let fanglepairs = map (\(a,_,b) -> (a,b)) fangles
  plotPaths [] $ map (take 100) [ walkpairs, fanglepairs ]
  -}
  return ()  
-}
