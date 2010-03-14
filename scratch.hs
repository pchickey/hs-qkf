{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction #-}

import Numeric.LinearAlgebra.Static
import Data.Packed.Static.Imports
import qualified Numeric.LinearAlgebra.Linear as Lin

type Quat = Vector D4 Double

qq :: Quat
qq = [$vec| 0, 0, 0, 1|]

ee :: Vector D3 Double
ee = [$vec| 1, 3, 5 |]

-- [ex]; Cross-product matrix of 3x1 column vector e, defined as
crossProdMat :: (Element a) => Vector D3 a -> Matrix (D3,D3) a
crossProdMat v = let e1 = v @> 0
                     e2 = v @> 1
                     e3 = v @> 2
    in [$mat| 0, -1*e3, e2; e3, 0, -1*e1; -1*e2, e1, 0 |]

type AttitudeMat = Matrix (D3, D3) Double

qre :: Quat -> Double
qre q = q @> 3

qvec :: Quat -> Vector D3 Double
qvec q = let q0 = q @> 0
             q1 = q @> 1
             q2 = q @> 2
        in [$vec| q0, q1, q2 |]

attMatOfQ :: Quat -> AttitudeMat
attMatOfQ qq = let q = qre qq 
                   ev = qvec qq
                   e = asColumn ev
                   et = trans e
                   ete = (et <> e) @@> (0,0)
                   ex = crossProdMat ev
               in Lin.scale (q*q - ete) (ident `atRows` d3) + 
                  Lin.scale 2  ( e <> et ) -
                  Lin.scale (2*q)  ex
