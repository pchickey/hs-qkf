-- Pat Hickey 05 Apr 10
{-# LANGUAGE QuasiQuotes #-}
import Numeric.LinearAlgebra.Static
import Control.Concurrent
import Qkf
import Cube
import SerialQKF

type CalSampleVar = SampleVar CalibratedMeasurment
type ThreeCals = (CalSampleVar, CalSampleVar, CalSampleVar)
type KFilter = (FilterState, RateEstimate)

main = do
  acccal   <- newEmptySampleVar
  magcal   <- newEmptySampleVar
  gyrocal  <- newEmptySampleVar
  filtered <- newEmptySampleVar
  
  forkIO $ sampleCalibrated (acccal, magcal, gyrocal)
  forkIO $ loopAndSend filtered (filterSamples (acccal, magcal, gyrocal)) (fszero, rezero)
  forkIO $ cubewith filtered

loopAndSend :: SampleVar a -> (a -> IO a) -> a -> IO ()
loopAndSend svar f init =
  let loop x = 
        f x >>= \result ->
        writeSampleVar svar result >>
        loop result
  in loop init


filterSamples :: ThreeCals -> KFilter ->  IO KFilter
filterSamples (acccal, magcal, gyrocal) (fstate, rstate) = 
 let dt = 0.03 in
 do
  latestacc <- readSampleVar acccal
  latestmag <- readSampleVar magcal
  latestgyro <- readSampleVar gyrocal
  let acc = toMeasurment latestacc
  let mag = toMeasurment latestmag
  let gyro = toMeasurment latestgyro  
  let fs'acc = measurmentUpdate acc fstate
  let fs'mag = measurmentUpdate mag fs'acc
  let rstate' = rateEstimateUpdate (toMeasurment latestgyro) dt rstate
  let fs'gyro = timePropogate rstate' fs'mag
  print fs'gyro
  print rstate'
  return (fs'gyro, rstate')

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


