-- Pat Hickey 05 Apr 10
import Control.Concurrent
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




