-- Pat Hickey 25 Mar 10
import Control.Concurrent
import QkfTest
import Qkf
import Cube


main = do
  fvar <- newEmptyMVar
  putMVar fvar (fszero, rezero) 
  forkIO (cubewith fvar)
  forkIO (iotest velocitytest fvar)
