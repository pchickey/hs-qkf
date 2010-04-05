-- Sensor frontend for Quaternion Kalman Filter
-- Pat Hickey 17 Mar 10
{-# LANGUAGE QuasiQuotes, NoMonomorphismRestriction, TypeOperators, FlexibleContexts 
    , ViewPatterns, ScopedTypeVariables, PatternGuards #-}
module SerialQKF where
import System.Hardware.Serialport
import Control.Monad hiding (forM, forM_)
import Data.Foldable -- has instance Foldable Maybe, so we can forM across Maybes
import Control.Monad.Loops
import Control.Concurrent
import Numeric.LinearAlgebra.Static
import Qkf

serialBegin :: IO SerialPort
serialBegin = do
  s <- openSerial "/dev/ftdi5" defaultSerialSettings { baudRate = B115200 }
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

data RawMeasurment = RM MeasurmentSource Double Double Double deriving (Show, Eq)

split :: String -> Char -> [String]
split [] _ = [""]
split (c:cs) delim 
  | c == delim = "" : rest
  | otherwise = ( c: head rest ) : tail rest
  where rest = split cs delim


parseLine :: [Char] -> Maybe RawMeasurment
parseLine cs = let prefix = take 2 cs 
               in case prefix of
                 "$A" -> rawOf Accelerometer $ drop 3 cs
                 "$M" -> rawOf Magnetometer $ drop 3 cs
                 "$G" -> rawOf Gyro $ drop 3 cs
                 _ -> Nothing
                

rawOf :: MeasurmentSource -> [Char] -> Maybe RawMeasurment
rawOf m s = let vs = split s ',' in
            case map readM vs of
              (Just x):(Just y):(Just z):[] ->  Just (RM m x y z)
              _ -> Nothing

-- A safer way to 'read' using reads. Use Maybe to avoid Prelude.read's no parse exception
readM :: (Read a) => String -> Maybe a
readM s | x:[] <- parse = (Just x)
        | otherwise = Nothing
 where parse = [x | (x,_) <- reads s]

{- unused, untested? ugly, at least.
vecOfCsv :: [Char] -> Vector D3 Double
vecOfCsv cs = let s = drop 3 cs
                  (x,y,z) = read s :: (Double,Double,Double)
                  h = (d1 >< d3) [ x, y, z ]
              in flatten h
-}

loopminmax (mins, maxs) f = 
  f (mins, maxs)                                                        >>= \r ->
  let ((minacc, minmag),(maxacc, maxmag)) = r in
  putStrLn ("acc- min: " ++ show minacc ++ " max: " ++ show maxacc ++
    " -mag- min: " ++ show minmag ++ " max: " ++ show maxmag)           >> 
  loopminmax r f

displayminsmaxs :: IO ()
displayminsmaxs = do
  s <- serialBegin
  let initmins = ((1024,1024,1024),(1024,1024,1024))
  let initmaxs = ((-1024,-1024,-1024),(-1024,-1024,-1024))
  loopminmax (initmins, initmaxs) (minmaxlines s)
  
minmaxlines s (mins, maxs) = do
  l <- getLineMaybe s
  case parseLine l of
    Just m@(RM Accelerometer _ _ _) -> 
      let (accmin, accmax) = newminmax (fst mins, fst maxs) m in
      return ((accmin, snd mins), (accmax, snd maxs))
    Just m@(RM Magnetometer _ _ _) -> 
      let (magmin, magmax) = newminmax (snd mins, snd maxs) m in
      return ((fst mins, magmin), (fst maxs, magmax))
    _ -> return (mins, maxs)

newminmax ((minx, miny, minz),(maxx, maxy, maxz)) (RM _ x y z) = (mins', maxs')
  where
    mins' = (min minx x, min miny y, min minz z)
    maxs' = (max maxx x, max maxy y, max maxz z)

offset :: MeasurmentSource -> (Double, Double, Double)
offset Accelerometer  = (0.0, 0.0, 0.0)
offset Magnetometer   = (-55.0, 2.0, 28.0)
offset Gyro           = (-1056.0, 403.0, 783.0) 

scalefactor :: MeasurmentSource -> (Double, Double, Double)
scalefactor Accelerometer  = ((-1)/540, (-1)/540, (-1)/540) -- Datasheet lied about sign?
scalefactor Magnetometer   = (1/250, (-1)/250, 1/250) -- Should align it with gyro right-handed coord system if datasheet was correct
scalefactor Gyro           = (gyroscale, gyroscale, gyroscale)
  where gyroscale =(pi/180) / 16.3 -- 16.3 LSB per deg/sec

data CalibratedMeasurment = CM MeasurmentSource Double Double Double deriving (Show,Eq)

offsetandscale :: RawMeasurment -> CalibratedMeasurment
offsetandscale (RM source x y z) = 
  CM source (scax * (x - offx)) (scay * (y - offy)) (scaz * (z - offz)) 
  where 
    (offx, offy, offz) = offset source
    (scax, scay, scaz) = scalefactor source

displaycalibrated = do
  s <- serialBegin
  forever $ do
    l <- getLineMaybe s
    forM_ (parseLine l) (print . offsetandscale)
    
