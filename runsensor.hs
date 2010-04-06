-- Pat Hickey 05 Apr 10
{-# LANGUAGE QuasiQuotes #-}
import Numeric.LinearAlgebra.Static
import Control.Concurrent
import Qkf
import QkfTest
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
  let acc = accMeasurment latestacc
  let mag = magMeasurment latestmag latestacc
  let gyro = gyroMeasurment latestgyro  
  let fs'acc = measurmentUpdate acc fstate
  let fs'mag = measurmentUpdate mag fs'acc
  let rstate' = rateEstimateUpdate gyro dt rstate
  let fs'gyro = timePropogate rstate' fs'mag
  
--  print $ body acc
--  print $ body mag
--  print $ ref mag

  print $ eulersOfQ $ q fs'gyro

  --print rstate'
  --putStrLn $ "before gyro " ++ (show $ eulersOfQ $ q fs'mag)
  --putStrLn $ "after gyro  " ++ (show $ eulersOfQ $ q fs'gyro)
  return (fs'gyro, rstate')

accMeasurment (CM Accelerometer x y z) =
  Measurment
    { source = Accelerometer 
    , body = [$vec| x, y, z |]
    , ref = [$vec| 0, 0, -1|]
    , meascov = magacccov }

-- Quick and dirty way to deal with magnetic declination:
-- make the reference vector at the same angle to down reference as the
-- magnetic measurment is to the accelleration measurment.
-- this is probably a really bad idea when there are accellerations,
-- so in the future add a low pass filter or track it in the filter 
magMeasurment (CM Magnetometer mx my mz) (CM Accelerometer ax ay az) =
  Measurment
    { source = Magnetometer 
    , body = magunit
    , ref = [$vec| refnorth, 0, refdown|]
    , meascov = magacccov }
  where
    accv = [$vec| ax, ay, az |]
    magv = [$vec| mx, my, mz |]
    accunit = accv / constant (fnorm accv) 
    magunit = magv / constant (fnorm magv) 
    declination = acos $ accunit <.> magunit
    refnorth = sin declination
    refdown = (-1) * cos declination

gyroMeasurment (CM Gyro x y z) = 
  Measurment
  { source = Gyro
  , body = [$vec| 0, 0, 0 |]
  --, body = [$vec| x, y, z |]
  , ref = [$vec| 0, 0, 0 |]
  , meascov = defaultcov }

                      
magacccov :: MeasurmentCovMat
magacccov = [$mat|0.001, 0, 0;
                   0, 0.001, 0;
                   0, 0, 0.001|]

