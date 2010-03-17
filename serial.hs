import System.Hardware.Serialport
import Control.Monad
import Control.Monad.Loops
import Control.Concurrent

serialBegin :: IO ()
serialBegin = do
  s <- openSerial "/dev/ttyUSB0" defaultSerialSettings { baudRate = B115200 }
  -- next 3 lines: toggle reset on Arduino
  setDTR s False
  threadDelay 500000
  setDTR s True
  forever (getLineMaybe s >>= putStrLn)

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



