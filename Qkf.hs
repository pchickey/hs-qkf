-- Quaternion Kalman Filter for attitude estimation
-- Pat Hickey 15 Mar 10
{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables #-}
module Qkf where
-- 9 out of 10 computer scientists recommend hmatrix-static
import Numeric.LinearAlgebra.Static

-- for deltaT and so on
type Time = Double
-- vec first {i j k}, then re
type Quat = Vector D4 Double
type QuatCovMat = Matrix (D4, D4) Double
-- Defined in paper, but we don't use it?
type AttitudeMat = Matrix (D3, D3) Double
-- All 3-space Measurments are zero-mean
type MeasurmentCovMat = Matrix (D3, D3) Double
type RateCovMat = Matrix (D3, D3) Double
-- 3-space vectors for acc,mag
type BodyVec = Vector D3 Double
type RefVec = Vector D3 Double
-- 3-space angular rate, measured in Body frame
type AngularRate = Vector D3 Double
-- Observations are of form 0 = Hnot*q
type ObservationMat = Matrix (D4,D4) Double
-- Observation noise maps 3space to quat space
type ZetaMat = Matrix (D4,D3) Double
-- Translate qspace to qspace
type TransitionMat = Matrix (D4,D4) Double


data FilterState = 
  FilterState{ q :: Quat
             , p :: QuatCovMat } deriving( Show, Eq )

data RateEstimate = 
  RateEstimate{ omega   :: AngularRate
              , qke :: RateCovMat
              , dt      :: Time } deriving( Show, Eq )

data MeasurmentSource = Accelerometer
                      | Magnetometer 
                      | Gyro deriving( Show, Eq )

data Measurment =
  Measurment{ source  :: MeasurmentSource
            , body    :: BodyVec
            , ref     :: RefVec
            , meascov :: MeasurmentCovMat }  deriving( Show, Eq )

-- sample vectors for using in ghci
qq :: Quat
qq = [$vec| 0, 0, 0, 1|]

ee :: Vector D3 Double
ee = [$vec| 1, 3, 5 |]

-- [ex]; Cross-product matrix of 3x1 column vector e, defined as
crossProdMat :: (Element a) => Vector D3 a -> Matrix (D3,D3) a
crossProdMat v = [$mat| 0, -1*e3, e2; 
                        e3, 0, -1*e1; 
                        -1*e2, e1, 0 |]
                  where e1 = v @> 0
                        e2 = v @> 1
                        e3 = v @> 2

qre :: Quat -> Double
qre q = q @> 3

qvec :: Quat -> Vector D3 Double
qvec q = [$vec| q0, q1, q2 |]
         where q0 = q @> 0
               q1 = q @> 1
               q2 = q @> 2
unitq :: Quat -> Quat 
unitq qq = qq / constant norm 
           where norm = sqrt $ (qq@>0)*(qq@>0)+(qq@>1)*(qq@>1)+(qq@>2)*(qq@>2)+(qq@>3)*(qq@>3)

attMatOfQ :: Quat -> AttitudeMat
attMatOfQ qq = liftMatrix (*constant  (q*q - e <.> e)) (ident `atRows` d3) + 
               liftMatrix (*constant 2)  (outer e e) -
               liftMatrix (*constant (2*q)) (crossProdMat e)
               where q = qre qq 
                     e = qvec qq

--negM :: (IntegerT (m :*: n)) => Matrix (m,n) Double -> Matrix (m,n) Double
--negM = liftMatrix (*constant (-1))

vecQRightMat :: Vector D3 Double -> Matrix (D4, D4) Double
vecQRightMat b = ((negboX <|> bo) <-> (negbot <|> [$mat| 0 |]))
                 where negboX = liftMatrix (*constant (-1)) (crossProdMat b)
                       bo = asColumn b
                       negbot = liftMatrix (*constant (-1)) (asRow b)

vecQLeftMat :: Vector D3 Double -> Matrix (D4, D4) Double
vecQLeftMat b = ((boX <|> bo) <-> (negbot <|> [$mat| 0 |]))
                 where boX = crossProdMat b
                       bo = asColumn b
                       negbot = liftMatrix (*constant (-1)) (asRow b)

observationMatOf :: RefVec -> BodyVec -> ObservationMat
observationMatOf r b = ( (negsx <|> asColumn d) <->
                         (negdt <|> [$mat|0|]))
                       where s = liftVector2 (*) (constant 0.5) (b + r)
                             d = liftVector2 (*) (constant 0.5) (b - r)
                             negsx = liftMatrix (*constant (-1)) (crossProdMat s)
                             negdt = liftMatrix (*constant (-1)) (asRow d)
zetaMatOf :: Quat -> ZetaMat
zetaMatOf qq = (ex + liftMatrix (* constant q) (ident `atRows` d3)) <-> 
               (liftMatrix (* constant (-1)) (asRow e))
               where q = qre qq
                     e = qvec qq
                     ex = crossProdMat e

transitionMatOf :: AngularRate -> Time -> TransitionMat
transitionMatOf w dt = expm omegadt
                       where omega = vecQRightMat w
                             omegadt = liftMatrix (* constant dt) omega

timePropogate :: RateEstimate -> FilterState -> FilterState
timePropogate r s = FilterState { q = unitq $ phi <> (q s)
                                , p = phi <> (p s) <> (trans phi) + qkq }
                                where phi = transitionMatOf (omega r) (dt r)
                                      zeta = zetaMatOf (q s)
                                      zetat = ((dt r)/2)^2
                                      qkq = liftMatrix (*constant zetat) (zeta <> (qke r) <> (trans zeta))

measurmentUpdate :: Measurment -> FilterState -> FilterState
measurmentUpdate m s = FilterState { q = unitq $ up <> (q s)
                                   , p = up <> (p s) <> (trans up) +
                                         k <> rq <> (trans k) }
                                   where i4 = ident `atRows` d4
                                         alpha = 0.001
                                         h = observationMatOf (ref m) (body m)
                                         ht = (trans h)
                                         zeta = zetaMatOf (q s)
                                         rq = (liftMatrix (*constant 0.25) 
                                                  (zeta <> (meascov m) <> (trans zeta))) + 
                                              (liftMatrix (*constant alpha) i4)
                                         sk = h <> (p s) <> ht + rq 
                                         k = (p s) <> ht <> (inv sk)  
                                         up = (i4 - k <> h)

rateEstimateUpdate :: Measurment -> Time -> RateEstimate -> RateEstimate
rateEstimateUpdate Measurment { source = Gyro, body = b } adt re = 
  RateEstimate { omega = (omega re) + constant k * residual
               , qke = (qke re)
               , dt = adt }
  where 
    k = 0.8
    residual = b - (omega re)

rateEstimateUpdate _ _ re = re 

rezero = RateEstimate { omega = [$vec|0,0,0|]
                      , dt = 0.02
                      , qke = liftMatrix (*constant 10) (atRows ident d3) }
fszero = FilterState { q = [$vec|0,0,0,1|]
                     , p = liftMatrix (*constant 5) (atRows ident d4) }



