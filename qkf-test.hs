-- Test of Quaternion Kalman Filter
-- Pat Hickey 19 Mar 10

{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables #-}
import Test.HUnit
import Control.Monad
import System.Random
import Numeric.LinearAlgebra.Static
import Graphics.Gnuplot.Simple
import Qkf

-- All Euler angles use the 313 convention
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

-- Make a quaternion from Euler angles. Deibel (66)
qOfEulers :: Eulers -> Quat
qOfEulers eulers = 
  [$vec| cphi * cthe * cpsi - sphi * cthe * spsi,
         cphi * cpsi * sthe + sphi * sthe * spsi,
         cphi * sthe * spsi - sphi * cpsi * sthe,
         cphi * cthe * spsi + cthe * cpsi * sphi |]
  where 
    cphi = cos . (/2) . phi   $ eulers 
    sphi = sin . (/2) . phi   $ eulers 
    cthe = cos . (/2) . theta $ eulers
    sthe = sin . (/2) . theta $ eulers 
    cpsi = cos . (/2) . psi   $ eulers
    spsi = sin . (/2) . psi   $ eulers

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

toVecMeasurment :: MeasurmentSource -> RefVec -> Eulers -> Measurment
toVecMeasurment s r e = 
  Measurment{ source = s
            , body = b
            , ref = r
            , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }
  where b = (matOfEulers e) <> r

toAccMeasurment = toVecMeasurment Accelerometer [$vec|0,0,-1|] 
toMagMeasurment = toVecMeasurment Magnetometer [$vec|1,0,0|] 

toGyroMeasurment :: Eulers -> WorldAngularRate-> Measurment
toGyroMeasurment e edot = 
  Measurment { source = Gyro
             , body = edot -- TODO -- bring to body frame from world frame
             , ref = [$vec|0,0,0|]
             , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }

-- Constant walk: keeps a constant derivative, for now zero
generateCWalk :: Eulers -> Time -> [Eulers]
generateCWalk e dt = (e:es)
                  where e' = advanceConst e ezero dt
                        es = generateCWalk e' dt

advanceConst :: Eulers -> WorldAngularRate -> Time -> Eulers
advanceConst angle angledot dt = angle

feedfilter :: [(Measurment, Measurment, Measurment)] -> (FilterState, RateEstimate) -> [(FilterState, RateEstimate)]
feedfilter ((ma, mm, mg):ms) (state,rate) = ((state,rate):states)
  where 
    s'acc = measurmentUpdate ma state
    s'mag = measurmentUpdate mm s'acc
    rate' = rateEstimateUpdate mg 0.02 rate
    s'gyro = timePropogate rate' s'mag
    states = feedfilter ms (s'gyro, rate')

statenorm = sumsq . q  
          where sumsq qq = (qq@>0)*(qq@>0) + (qq@>1)*(qq@>1) + (qq@>2)*(qq@>2) + (qq@>3)*(qq@>3)
stateqs fs = map (`map` fs) [ q0 . q . fst , q1 . q . fst, q2 . q . fst, q3 . q . fst]

-------------------------------------------------------------
statictest = do
  let e = [$vec| pi/4, pi/12, -pi/8 |]
  let edot = [$vec| 0,0,0 |]
  let f = feedfilter (repeat 
                      (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e edot)) 
                     (fszero, rezero)
  plotLists [] $ stateqs $ take 10 f
  --plotLists [] $ ( take 10  f ) 
 
-------------------------------------------------------------
{-
consttest = do
  let e = Eulers { alpha = Angle { a = 0, da = pi/4 }
                 , beta  = Angle { a = pi/6, da = 0 }
                 , gamma = Angle { a = pi/3, da = 0 } }
  let walk = generateCWalk e 0.05 
  let meas = map (\e -> (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e)) 
  let f = feedfilter (meas walk) (fszero, rezero) 
  plotLists [] $ stateqs $ take 50 f
  
  plotLists [] $ anglelists ( take 10  f ) 
  --let walkpairs = map (\e -> ( (a $ alpha e), (a $ beta e) )) walk
  --let etuple = map ((\(_,b,c) -> (-1*c,-1*b)) . qtoEtuple . q . fst) f
  --plotPaths [] $ map (take 5) [walkpairs, etuple]
-}
--- rosetta code provided a gaussian snippet, somewhat refactored here
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
