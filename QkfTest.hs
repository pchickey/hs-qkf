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
-- Project Modules:
import Quaternion
import Eulers313
import Qkf
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

feedfilter' :: Time -> [(Measurment, Measurment, Measurment)] -> (FilterState, RateEstimate) -> [(FilterState, RateEstimate)]
feedfilter' dt ((ma, mm, mg):ms) (state,rate) = ((state,rate):states)
  where 
    s'acc = measurmentUpdate ma state
    s'mag = measurmentUpdate mm s'acc
    rate' = rateEstimateUpdate mg dt rate
    s'gyro = timePropogate' rate' s'mag
    states = feedfilter dt ms (s'gyro, rate')


statenorm = sumsq . q  
          where sumsq qq = (qq@>0)*(qq@>0) + (qq@>1)*(qq@>1) + (qq@>2)*(qq@>2) + (qq@>3)*(qq@>3)

-- The following functions demonstrate my ignorance of Haskell:
qaccs = [ q0, q1, q2, q3 ]
stateqs fs = map (\qacc -> map (qacc . q . fst) fs) qaccs
justqs  qs = map (\qacc -> map qacc qs) qaccs
angleaccs = [phi, theta, psi]
stateangles as = map (\angleacc -> map (angleacc . eulersOfQ . q  . fst) as) angleaccs 
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
 
  let f = feedfilter' dt (meas walk edots) (fszero, rezero) 
  let fs = take 50 f
  let ws = take 50 walk

  plotLists [Title "Euler Angles"] $ stateangles fs ++ angles ws
  plotLists [Title "Quat Components"] $ stateqs fs ++ (justqs $ map qOfEulers ws)
--  return f

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
