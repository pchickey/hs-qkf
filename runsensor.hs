-- Pat Hickey 05 Apr 10
{-# LANGUAGE QuasiQuotes #-}
import Numeric.LinearAlgebra.Static
import Control.Concurrent
import Qkf
import QkfTest
import Cube
import SerialQKF
import Graphics.Gnuplot.Simple

type CalSampleVar = SampleVar CalibratedMeasurment
type ThreeCalSamples = (CalSampleVar, CalSampleVar, CalSampleVar)
type KFilter = (FilterState, RateEstimate)
type ThreeMeas = (Measurment, Measurment, Measurment)
type ThreeCals = (CalibratedMeasurment, CalibratedMeasurment, CalibratedMeasurment)

main = do
  acccal   <- newEmptySampleVar
  magcal   <- newEmptySampleVar
  gyrocal  <- newEmptySampleVar
  filtered <- newEmptySampleVar
  reslog <- newChan

  t1 <- forkIO $ sampleCalibrated (acccal, magcal, gyrocal)
  t2 <- forkIO $ loopAndSend filtered (filterSamples (acccal, magcal, gyrocal)) (fszero, rezero) reslog
  t3 <- forkIO $ cubewith filtered >> killThread t1 >> killThread t2
  
  plotresults reslog
  print "done plotting results"

unzip3vec (v:vs) = (v1:v1s , v2:v2s, v3:v3s)
  where v1 = v @> 0
        v2 = v @> 1
        v3 = v @> 2
        (v1s, v2s, v3s) = unzip3vec vs
unzip3vec [] = ([], [], [])

plotresults rchan = do
  print "getting chan contents"
  rs <- getChanContents rchan 
  let (outs, ins) = unzip $ take 100 rs
  let (fstates, restates) = unzip outs
  let (accs, mags, gyros) = unzip3 ins
  let (axs, ays, azs) = unzip3vec (map body accs)
  let (mxs, mys, mzs) = unzip3vec (map body mags)
  let (gxs, gys, gzs) = unzip3vec (map body gyros)
  print "beginning plots"
  plotLists [Title "Accelerometer"] $ [axs, ays, azs]
  plotLists [Title "Magnetometer"] [mxs, mys, mzs]


-- loopAndSend :: SampleVar b -> (a -> IO a) -> a -> Chan a -> IO ()
loopAndSend svar f init logchan =
  let loop x = 
        f x >>= \result ->
        writeSampleVar svar (fst result) >> -- now we're sending (outs, ins) as result
        writeChan logchan result >>         -- forward outs to svar, log (outs, ins)
        loop (fst result)
  in loop init


filterSamples :: ThreeCalSamples -> KFilter ->  IO (KFilter, ThreeMeas)
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
  return ((fs'gyro, rstate'), (acc, mag, gyro))

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

