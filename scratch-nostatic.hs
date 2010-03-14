
import Numeric.LinearAlgebra

type Vec = Vector Double
type Mat = Matrix Double
type Quat = Vec

qq :: Quat
qq = fromList [ 0, 0, 0, 1]

ee :: Vec
ee = fromList [ 1, 3, 5]

-- [ex]; Cross-product matrix of 3x1 column vector e, defined as
crossProdMat :: Vec -> Mat
crossProdMat v = let e1 = v @> 0
                     e2 = v @> 1
                     e3 = v @> 2
    in (3><3) [ 0, -1*e3, e2
              , e3, 0, -1*e1
              , -1*e2, e1, 0]

type AttitudeMat = Mat

qre :: Quat -> Double
qre q = q @> 3

qvec :: Quat -> Vec
qvec q = let q0 = q @> 0
             q1 = q @> 1
             q2 = q @> 2
        in fromList [ q0, q1, q2 ]

attMatOfQ :: Quat -> AttitudeMat
attMatOfQ qq = let q = qre qq 
                   e = qvec qq
                   ex = crossProdMat e
               in scalar (q*q - dot e e) * (ident 3) + 
                  2* ( outer e e ) -
                  scalar (2*q) * ex
