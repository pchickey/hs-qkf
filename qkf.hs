-- Quaternion Kalman Filter for attitude estimation
-- Pat Hickey 15 Mar 10

{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables #-}
-- 9 out of 10 computer scientists recommend hmatrix-static
import Numeric.LinearAlgebra.Static
import Data.Packed.Static.Imports

type Quat = Vector D4 Double
type Vec3 = Vector D3 Double
type AttitudeMat = Matrix (D3, D3) Double

type MagCovMat = Matrix (D3, D3) Double
type AccCovMat = Matrix (D3, D3) Double
type GyroCovMat = Matrix (D3, D3) Double

type BodyFrameVec = Vec3
type RefFrameVec = Vec3
-- Observations are of form 0 = Hnot*q
type ObservationMat = Matrix (D4,D4) Double

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
        
attMatOfQ :: Quat -> AttitudeMat
attMatOfQ qq = liftMatrix (*constant  (q*q - e <.> e)) (ident `atRows` d3) + 
               liftMatrix (*constant 2)  (outer e e) -
               liftMatrix (*constant (2*q)) (crossProdMat e)
               where q = qre qq 
                     e = qvec qq

--negM :: (IntegerT (m :*: n)) => Matrix (m,n) Double -> Matrix (m,n) Double
--negM = liftMatrix (*constant (-1))

vecQRightMat :: Vector D3 Double -> Matrix (D4, D4) Double
vecQRightMat b = fromBlocks33to44 negboX bo  negbot [$mat| 0 |]
                 where negboX = liftMatrix (*constant (-1)) (crossProdMat b)
                       bo = asColumn b
                       negbot = liftMatrix (*constant (-1)) (asRow b)

vecQLeftMat :: Vector D3 Double -> Matrix (D4, D4) Double
vecQLeftMat b = fromBlocks33to44 boX bo  negbot [$mat| 0 |]
                 where boX = crossProdMat b
                       bo = asColumn b
                       negbot = liftMatrix (*constant (-1)) (asRow b)
-- ugly version:
fromBlocks33to44 :: Matrix (D3,D3) Double -> Matrix (D3,D1) Double -> Matrix (D1,D3) Double -> Matrix (D1,D1) Double -> Matrix (D4, D4) Double
fromBlocks33to44  (viewMat -> [$mat|a, b, c;
                                    d, e, f;
                                    g, h, i|])
                  (viewMat -> [$mat|j; k; l|])
                  (viewMat -> [$mat|m, n, o|])
                  (viewMat -> [$mat|p|]) = [$mat|a, b, c, j;
                                                  d, e, f, k;
                                                  g, h, i, l;
                                                  m, n, o, p|]
-- less ugly version which doesn't work:
-- fromBlocks33to44' :: Matrix (D3,D3) Double -> Matrix (D3,D1) Double -> Matrix (D1,D3) Double -> Matrix (D1,D1) Double -> Matrix (D4, D4) Double
-- fromBlocks33to44' a b c d = fromBlocksU [[ a, b],[ c, d]] `atShape` (d4, d4)

-- Use the proper operators:
fromBlocks33to44' :: Matrix (D3,D3) Double -> Matrix (D3,D1) Double -> Matrix (D1,D3) Double -> Matrix (D1,D1) Double -> Matrix (D4, D4) Double
fromBlocks33to44' a b c d = (( a <|> b ) <-> ( c <|> d ))

observationMatOf :: RefFrameVec -> BodyFrameVec -> ObservationMat
observationMatOf r b = (fromBlocks33to44  negsx (asColumn d)  
                                          negdt [$mat|0|])
                       where s = liftVector2 (*) (constant 0.5) (b + r)
                             d = liftVector2 (*) (constant 0.5) (b - r)
                             negsx = liftMatrix (*constant (-1)) (crossProdMat s)
                             negdt = (liftMatrix (*constant (-1)) (asRow d))







