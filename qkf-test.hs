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

data Angle = 
  Angle { a  :: Double
        , da :: Double } deriving (Show, Eq)-- angle and its derivative
data Eulers = 
  Eulers { alpha :: Angle
         , beta  :: Angle
         , gamma :: Angle } deriving (Show, Eq)

ezero = Eulers { alpha = Angle { a = 0, da = 0 }
               , beta  = Angle { a = 0, da = 0 }
               , gamma = Angle { a = 0, da = 0 } }
azero = Angle { a = 0, da = 0 }

type RotMatrix = Matrix (D3,D3) Double

matOfEulers :: Eulers -> RotMatrix
matOfEulers eulers = [$mat|c1*c3 - c2*s1*s3, (-1)*c3*s1 - c1*c2*s3, s2*s3;
                           c2*c3*s1 + c1*s3, c1*c2*c3 - s1*s3,      (-1)*c3*s2;
                           s1*s2,            c1*s2,                 c2|]
                     where c1 = cos $ a $ alpha  eulers
                           s1 = sin $ a $ alpha  eulers
                           c2 = cos $ a $ beta   eulers
                           s2 = sin $ a $ beta   eulers
                           c3 = cos $ a $ gamma  eulers
                           s3 = sin $ a $ gamma  eulers

rateOfEulers :: Eulers -> Vector D3 Double
rateOfEulers eulers = [$vec| alphadot * sinbeta * singamma + betadot * cosgamma,
                             alphadot * sinbeta * cosgamma - betadot * singamma,
                             alphadot * cosbeta + gammadot |]
                      where alphadot = da $ alpha  eulers
                            betadot  = da $ beta   eulers
                            gammadot = da $ gamma  eulers
                            sinbeta  = sin $ a $ beta   eulers
                            cosbeta  = cos $ a $ beta   eulers
                            singamma = sin $ a $ gamma  eulers
                            cosgamma = cos $ a $ gamma  eulers

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
                    where (r1, g1) = random g
                          (r2, g2) = random g1

advanceRandAngle :: (RandomGen g) => Double -> Angle -> Time -> g -> (Angle, g)
advanceRandAngle sm ang dt g = ( Angle { a = a', da = da' }, g')
                               where (gauss1, g') = gauss (30*pi/180) g
                                     da' = sm * (da ang) + (1-sm) * gauss1 
                                     a' = (a ang) + da' * dt
                            
advanceEulerR :: (RandomGen g) => Eulers -> g -> (Eulers, g)
advanceEulerR e g = ( Eulers { alpha = alpha', beta = beta', gamma = (gamma e) } , g2 )
                    where (alpha', g1) = advanceRandAngle smoothness (alpha e) 0.02 g
                          (beta', g2) = advanceRandAngle smoothness (beta  e) 0.02 g1
                          smoothness = 0.95

generateRWalk :: (RandomGen g) => Eulers -> g -> [Eulers]
generateRWalk x g = (v:vs)
                   where (v, g') = advanceEulerR x g
                         vs = generateRWalk v g'

toVecMeasurment :: (MeasurmentSource, RefVec) -> Eulers -> Measurment
toVecMeasurment (s, r) e = Measurment{ source = s
                                     , body = b
                                     , ref = r
                                     , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }
                           where b = (matOfEulers e) <> r
toAccMeasurment = toVecMeasurment (Accelerometer, [$vec|0,0,(-1)|])
toMagMeasurment = toVecMeasurment (Magnetometer, [$vec|1,0,0|])
toGyroMeasurment e = Measurment { source = Gyro
                                , body = [$vec|rx,ry,rz|]
                                , ref = [$vec|0,0,0|]
                                , meascov = liftMatrix (* constant 0.001) (atRows ident d3) }
                     where rx = da $ alpha e
                           ry = da $ beta  e
                           rz = da $ gamma e

qtoEtuple :: Quat -> (Double,Double,Double) -- quick n dirty; may be based on wrong convention
qtoEtuple q = (phi, theta, psi)
            where q0 = q @> 3; q1 = q @> 0; q2 = q @> 1; q3 = q @> 2
                  phi = atan2 (2*(q0*q1+q2*q3)) (1-2*(q1*q1+q2+q2))
                  theta = asin (2*(q0*q2-q3*q1))
                  psi = atan2 (2*(q0*q3+q1*q2)) (1-2*(q2*q2+q3*q3))

feedfilter :: [(Measurment, Measurment, Measurment)] -> (FilterState, RateEstimate) -> [(FilterState, RateEstimate)]
feedfilter ((ma, mm, mg):ms) (state,rate) = ((state,rate):states)
                                      where s'acc = measurmentUpdate ma state
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
stateqs fs = [ map stateq0 fs, map stateq1 fs, map stateq2 fs, map stateq3 fs , map statenorm fs]

staticmeas m = (m:ms) where ms = staticmeas m
statictest = do
  let e = Eulers { alpha = Angle { a = pi/4, da = 0 }
                 , beta =  Angle { a = pi/3, da = 0 }
                 , gamma = Angle { a = -pi/10, da = 0 } }
  let f = feedfilter (staticmeas 
                      ( toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e)) 
                     (fszero, rezero)
  plotLists [] $ stateqs $ take 10 f


rtest = do
  g <- getStdGen
  let walk = generateRWalk ezero g
  let walkpairs = map (\e -> ( (a $ alpha e), (a $ beta e) )) walk
  --plotPath [] $ take 100 walkpairs
  let meas = map (\e -> (toAccMeasurment e, toMagMeasurment e, toGyroMeasurment e)) 
  let f = feedfilter (meas walk) (fszero, rezero)
  
  --plotLists [] $ stateqs $ take 100 f

  -- fangles isn't producing anything that makes much sense.
  let fangles = map (qtoEtuple . q . fst) f
  let fanglepairs = map (\(b,_,a) -> (a,b)) fangles
  plotPaths [] $ map (take 100) [ walkpairs, fanglepairs ]
  return ()
