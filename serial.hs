-- Sensor frontend for Quaternion Kalman Filter
-- Pat Hickey 17 Mar 10
{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables #-}
module SerialQKF where
import System.Hardware.Serialport
import Control.Monad
import Control.Monad.Loops
import Control.Concurrent
import Numeric.LinearAlgebra.Static
import Qkf

serialBegin :: IO SerialPort
serialBegin = do
  s <- openSerial "/dev/ttyUSB0" defaultSerialSettings { baudRate = B115200 }
  -- next 3 lines: toggle reset on Arduino
  setDTR s False
  threadDelay 500000
  setDTR s True
  return s

-- from Tackling the Awkward Squad 
getLineMaybe :: SerialPort -> IO [Char] 
getLineMaybe s = do { c <- recvChar s;
                      case c of
                      Just '\n' -> return []
                      Just a    -> do { cs <- getLineMaybe s;
                                      return (a:cs) }
                      Nothing   -> do { cs <- getLineMaybe s;
                                      return(cs) }
                    }

defaultcov :: MeasurmentCovMat
defaultcov = [$mat|0.001, 0, 0;
                   0, 0.001, 0;
                   0, 0, 0.001|]

parseLine :: [Char] -> Maybe Measurment
parseLine cs = let prefix = take 2 cs 
               in case prefix of
               "$A" -> Just Measurment { source = Accelerometer
                                       , body =  vecOfCsv cs
                                       , ref = [$vec|0,0,(-1.0)|]
                                       , meascov = defaultcov}
               "$M" -> Just Measurment { source = Magnetometer
                                       , body =  vecOfCsv cs 
                                       , ref = [$vec|1.0,0,0|]
                                       , meascov = defaultcov}
               "$G" -> Just Measurment { source = Gyro
                                       , body =  vecOfCsv cs
                                       , ref = [$vec|0,0,0|]
                                       , meascov = defaultcov}
               _ -> Nothing

vecOfCsv :: [Char] -> Vector D3 Double
vecOfCsv cs = let s = drop 3 cs
                  (x,y,z) = read s :: (Double,Double,Double)
                  h = (d1 >< d3) [ x, y, z ]
              in flatten h


main :: IO ()
main = do
  s <- serialBegin
  forever $ do
    l <- getLineMaybe s
    case parseLine l of
      Just a -> do { putStrLn (show a) }
      Nothing -> return ()

