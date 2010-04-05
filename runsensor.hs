-- Pat Hickey 05 Apr 10
{-# LANGUAGE QuasiQuotes #-}
import Numeric.LinearAlgebra.Static
import Control.Concurrent
import Control.Monad
import Qkf
import Cube
import SerialQKF

main = do
  acccal <- newEmptySampleVar
  magcal <- newEmptySampleVar
  gyrocal <- newEmptySampleVar
  forkIO $ sampleCalibrated (acccal, magcal, gyrocal)

  fvar <- newEmptyMVar
  putMVar fvar (fszero, rezero) 
  forkIO $ cubewith fvar

filterSamples (acccal, magcal, gyrocal) fvar = 
 let dt = 0.03 in
 forever $ do
  latestacc <- readSampleVar acccal
  latestmag <- readSampleVar magcal
  latestgyro <- readSampleVar gyrocal
  (fstate, rstate) <- takeMVar fvar
  let s'acc = measurmentUpdate latestacc fstate
  let s'mag = measurmentUpdate latestmag s'acc
  let rstate' = rateEstimateUpdate latestgyro dt rstate
  let s'gyro = timePropogate rstate' s'mag
  putMVar fvar (fstate, rstate)

toMeasurment :: CalibratedMeasurment -> Measurment
toMeasurment (CM Accelerometer x y z) =
  Measurment
    { source = Accelerometer 
    , body = [$vec| x, y, z |]
    , ref = [$vec| 0, 0, -1|]
    , meascov = defaultcov }
toMeasurment (CM Magnetometer x y z) =
  Measurment
    { source = Magnetometer 
    , body = [$vec| x, y, z |]
    , ref = [$vec| 1, 0, 0|]
    , meascov = defaultcov }



