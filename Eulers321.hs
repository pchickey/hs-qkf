
-- A set of functions for euler angles and quaternion conversions 
-- which use the 321 convention, as described in 
-- "Representing Attitude: Euler Angles, Unit QUaternions, and Rotation Vectors" James Diebel 2006

-- Pat Hickey 13 Apr 10

{-# LANGUAGE QuasiQuotes #-}
module Eulers321 where
import Debug.Trace
import Numeric.LinearAlgebra.Static
import Quaternion

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

-- Make a rotation matrix from Euler angles. Deibel (449)
matOfEulers :: Eulers -> RotMatrix
matOfEulers eulers = 
  [$mat| cphi*cthe , sphi*cpsi + cphi*sthe*spsi , sphi*spsi - cphi*sthe*cpsi 
       ;-sphi*cthe , cphi*cpsi - sphi*sthe*spsi , cphi*spsi + sphi*sthe*cpsi
       ;  sthe     ,-spsi*cthe                  , cpsi*cthe                  |]
  where
  cphi = cos . phi   $ eulers
  sphi = sin . phi   $ eulers
  cthe = cos . theta $ eulers
  sthe = sin . theta $ eulers
  cpsi = cos . psi   $ eulers
  spsi = sin . psi   $ eulers

-- Make a quaternion from Euler angles. Deibel (459), except re-ordered with real component last
-- to fit type Quat convention.
qOfEulers :: Eulers -> Quat
qOfEulers eulers =
  [$vec| 
         cphi * cthe * spsi + sphi * cpsi * sthe,
         cphi * cpsi * sthe - sphi * cthe * spsi,
         cphi * sthe * spsi + cthe * cpsi * sphi,
         cphi * cthe * cpsi - sphi * sthe * spsi
         |]
  where
  cphi = cos . (/2) . phi   $ eulers
  sphi = sin . (/2) . phi   $ eulers
  cthe = cos . (/2) . theta $ eulers
  sthe = sin . (/2) . theta $ eulers
  cpsi = cos . (/2) . psi   $ eulers
  spsi = sin . (/2) . psi   $ eulers

-- Deibel (452)
eulersOfQ :: Quat -> Eulers
eulersOfQ q =
  [$vec| atan2 ((-2)*qq1*qq2 + 2*qq0*qq3) (qq1*qq1 + qq0*qq0 - qq3*qq3 - qq2*qq2)
       , asinLimited (2*qq1*qq3 + 2*qq0*qq2)
       , atan2 ((-2)*qq2*qq3 + 2*qq0*qq1) (qq3*qq3 - qq2*qq2 - qq1*qq1 + qq0*qq0) |]
  where 
  qq0 = q0 q; qq1 = q1 q; qq2 = q2 q; qq3 = q3 q
  asinLimited x = if (abs x) < 1.0 then asin x else asin (signum x) -- v small numeric overflows: 
                                                                    -- keep args to asin in [-1,1]
-- Euler Angle Rate Matrix
-- omega = E(u) * udot  where omega is body fixed angular rates, u is euler angles
-- Deibel (453)
type Emat = Matrix (D3,D3) Double
-- Deibel (453)
ematOfEulers :: Eulers -> Emat
ematOfEulers e = [$mat| sthe      , 0    , 1
                      ;-spsi*cthe , cpsi , 0
                      ; cpsi*cthe , spsi , 0 |]
  where
  cthe = cos . theta $ e
  sthe = sin . theta $ e
  cpsi = cos . psi   $ e
  spsi = sin . psi   $ e


