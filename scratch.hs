{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction #-}
-- 9 out of 10 computer scientists recommend hmatrix-static
import Numeric.LinearAlgebra.Static
import Data.Packed.Static.Imports

type Quat = Vector D4 Double

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
qvec q = let q0 = q @> 0
             q1 = q @> 1
             q2 = q @> 2
        in [$vec| q0, q1, q2 |]

type AttitudeMat = Matrix (D3, D3) Double
attMatOfQ :: Quat -> AttitudeMat
attMatOfQ qq = liftMatrix (*constant  (q*q - e <.> e)) (ident `atRows` d3) + 
               liftMatrix (*constant 2)  (outer e e) -
               liftMatrix (*constant (2*q)) (crossProdMat e)
               where q = qre qq 
                     e = qvec qq
               
