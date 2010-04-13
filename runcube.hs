-- Pat Hickey 25 Mar 10
import Control.Concurrent
import QkfTest
import Qkf
import Cube


main = do
  fvar <- newEmptySampleVar
  forkIO (cubewith fvar)
  testresults <- velocitytest
  forkIO (iotest testresults fvar)
