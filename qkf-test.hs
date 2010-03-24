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

v1 = [$vec| 0, 1, 2, 3 |]
v2 = [$vec| 1, 2, 1, 0 |]

-- All Euler angles use the 313 convention
-- see "Representing Attitude: Euler Angles, Unit QUaternions, and Rotation Vectors" James Diebel 2006
-- for reference on all conversions.
type Angle = Float
data Eulers = 
  Eulers { phi   :: Angle
         , theta :: Angle
         , psi   :: Angle } deriving (Show, Eq)

ezero = Eulers { phi = 0.0, theta = 0.0, psi = 0.0 }

type RotMatrix = Matrix (D3,D3) Double
matOfEulers :: Eulers -> RotMatrix
matOfEulers eulers = [$mat| cphi*cpsi - sphi*cthe*spsi,  cphi*spsi + sphi*cthe*cpsi,  sphi*sthe;
                            -sphi*cpsi - cphi*cthe*spsi,-sphi*spsi + cphi*cthe*cpsi, cphi*sthe;
                            sthe*spsi,                   sthe*cpsi,                   cthe      |]
  where 
    cphi = cos . phi $ eulers
    sphi = sin . phi $ eulers
    cthe = cos . theta $ eulers
    sthe = sin . theta $ eulers
    cpsi = cos . psi $ eulers
    spsi = sin . psi $ eulers
{- this is currently complete rubbish.
rateOfEulers :: Eulers -> Vector D3 Double
rateOfEulers eulers = [$vec| gammadot * cosbeta * cosalpha + betadot * sinalpha,
                             betadot * cosalpha  - gammadot * sinalpha * cosbeta,
                             gammadot * sinbeta + alphadot |]
                      where alphadot = da . alpha $ eulers
                            betadot  = da . beta  $ eulers
                            gammadot = da . gamma $ eulers
                            sinalpha = sin . a . alpha $ eulers
                            cosalpha = cos . a . alpha $ eulers
                            sinbeta  = sin . a . beta  $ eulers
                            cosbeta  = cos . a . beta  $ eulers
                            singamma = sin . a . gamma $ eulers
                            cosgamma = cos . a . gamma $ eulers
-}
-- just to get the program to typecheck:
rateOfEulers _ = [$vec| 0, 0, 0]

generateCWalk :: Eulers -> Time -> [Eulers]
generateCWalk e dt = (e:es)
                  where e' = advanceConst e dt
                        es = generateCWalk e' dt

advanceConst :: Eulers -> Time -> Eulers
advanceConst eulers t = Eulers { alpha = advanceAngle (alpha eulers) t
                               , beta  = advanceAngle (beta eulers) t
                               , gamma = advanceAngle (gamma eulers) t }

advanceAngle :: Angle -> Time -> Angle
advanceAngle orig t = Angle { a = (a orig) + (da orig) * t
                            , da = (da orig) }

-- rosetta code provided a gaussian snippet, somewhat refactored
gauss :: (RandomGen g) => Double -> g -> (Double, g)
gauss sigma g = ( sigma * sqrt (-2 * log r1) * cos ( 2 * pi * r2 )
                , g2 ) 
  where 
    (r1, g1) = random g
    (r2, g2) = random g1

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

toVecMeasurment :: (MeasurmentSource, RefVec) -> Eulers -> Measurment
toVecMeasurment (s, r) e = 
  Measurment{ source = s
            , body = b
            , ref = r
            , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }
  where b = (matOfEulers e) <> r

toAccMeasurment = toVecMeasurment (Accelerometer, [$vec|0,0,(-1)|])
toMagMeasurment = toVecMeasurment (Magnetometer, [$vec|1,0,0|])

toGyroMeasurment e = 
  Measurment { source = Gyro
             , body = rateOfEulers e
             , ref = [$vec|0,0,0|]
             , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }

feedfilter :: [(Measurment, Measurment, Measurment)] -> (FilterState, RateEstimate) -> [(FilterState, RateEstimate)]
feedfilter ((ma, mm, mg):ms) (state,rate) = ((state,rate):states)
  where 
    s'acc = measurmentUpdate ma state
    s'mag = measurmentUpdate mm s'acc
    rate' = rateEstimateUpdate mg 0.02 rate
    s'gyro = timePropogate rate' s'mag
    states = feedfilter ms (s'gyro, rate')

stateq0 = (@>0) . q . fst
stateq1 = (@>1) . q . fst
stateq2 = (@>2) . q . fst
stateq3 = (@>3) . q . fst
statenorm = sumsq . q . fst 
          where sumsq qq = (qq@>0)*(qq@>0) + (qq@>1)*(qq@>1) + (qq@>2)*(qq@>2) + (qq@>3)*(qq@>3)
stateqs fs = map (`map` fs) [ stateq0, stateq1, stateq2, stateq3]

anglelists f = [ map (qhead . q . fst) f
               , map (qatt  . q . fst) f
               , map (qbank . q . fst) f ]
-------------------------------------------------------------
statictest = do
  let e = Eulers { alpha = Angle { a = pi/12, da = 0 }
                 , beta =  Angle { a = pi/12, da = 0 }
                 , gamma = Angle { a = pi/12, da = 0 } }
  let f = feedfilter (repeat 
                      ( toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e)) 
                     (fszero, rezero)
  plotLists [] $ stateqs $ take 10 f
  plotLists [] $ anglelists ( take 10  f ) 
 
-------------------------------------------------------------
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

-----------------------------------------------------------------------
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
