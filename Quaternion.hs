module Quaternion where
import Numeric.LinearAlgebra.Static
type Quat = Vector D4 Double

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
