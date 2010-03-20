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
                            
advanceEuler :: (RandomGen g) => Eulers -> g -> (Eulers, g)
advanceEuler e g = ( Eulers { alpha = alpha', beta = beta', gamma = (gamma e) } , g2 )
                  where (alpha', g1) = advanceRandAngle smoothness (alpha e) 0.02 g
                        (beta', g2) = advanceRandAngle smoothness (beta  e) 0.02 g1
                        smoothness = 0.95

generateWalk x g = (v:vs)
                   where (v, g') = advanceEuler x g
                         vs = generateWalk v g'

