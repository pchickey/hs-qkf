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
  forkIO $ forever $ filterSamples (acccal, magcal, gyrocal) fvar
  
  forkIO $ cubewith fvar


filterSamples (acccal, magcal, gyrocal) fvar = 
 let dt = 0.03 in
 forever $ do
  latestacc <- readSampleVar acccal
  latestmag <- readSampleVar magcal
  latestgyro <- readSampleVar gyrocal
  (fstate, rstate) <- takeMVar fvar
  let acc = toMeasurment latestacc
  let mag = toMeasurment latestmag
  let gyro = toMeasurment latestgyro  
  let fs'acc = measurmentUpdate acc fstate
  let fs'mag = measurmentUpdate mag fs'acc
  let rstate' = rateEstimateUpdate (toMeasurment latestgyro) dt rstate
  let fs'gyro = timePropogate rstate' fs'mag
  print fs'gyro
  print rstate'
  putMVar fvar (fs'gyro, rstate')
  threadDelay 50

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
toMeasurment (CM Gyro x y z) = 
  Measurment
  { source = Gyro
  , body = [$vec| x, y, z |]
  , ref = [$vec| 0, 0, 0 |]
  , meascov = defaultcov }


