-- Credit where credit is due:
-- I copied the cube code, more or less wholesale, from Mark J Kilgard.
-- http://gist.github.com/309262
-- http://twoguysarguing.wordpress.com/2010/02/20/opengl-and-haskell/
-- Maybe one day I'll have the luxury of learning OpenGL properly, but not tonight!
-- Pat Hickey, 25 Mar 10

module Cube (cubewith) where
import Control.Concurrent
import Graphics.Rendering.OpenGL
import Graphics.UI.GLUT
import Qkf
import QkfTest

n :: [Normal3 GLfloat]
n = [(Normal3 (-1.0) 0.0 0.0),
     (Normal3 0.0 1.0 0.0),
     (Normal3 1.0 0.0 0.0),
     (Normal3 0.0 (-1.0) 0.0),
     (Normal3 0.0 0.0 1.0),
     (Normal3 0.0 0.0 (-1.0))]

faces :: [[Vertex3 GLfloat]]
faces = [[(v 0), (v 1), (v 2), (v 3)],
         [(v 3), (v 2), (v 6), (v 7)],
         [(v 7), (v 6), (v 5), (v 4)],
         [(v 4), (v 5), (v 1), (v 0)],
         [(v 5), (v 6), (v 2), (v 1)],
         [(v 7), (v 4), (v 0), (v 3)]]

v :: Int -> Vertex3 GLfloat
v x = Vertex3 v0 v1 v2
    where v0
              | x == 0 || x == 1 || x == 2 || x == 3 = -1
              | x == 4 || x == 5 || x == 6 || x == 7 = 1
          v1
              | x == 0 || x == 1 || x == 4 || x == 5 = -1
              | x == 2 || x == 3 || x == 6 || x == 7 = 1
          v2
              | x == 0 || x == 3 || x == 4 || x == 7 = 1
              | x == 1 || x == 2 || x == 5 || x == 6 = -1

drawBox :: IO ()
drawBox = let nfaces = zip n faces
          in do mapM (\(n, [v0, v1, v2, v3]) -> do
                        renderPrimitive Quads $ do
                          normal n
                          vertex v0
                          vertex v1
                          vertex v2
                          vertex v3) nfaces
                return ()

display :: DisplayCallback
display = do
  clear [ColorBuffer, DepthBuffer]
  putStrLn "DisplayCallback Called"
  drawBox
  swapBuffers

lightDiffuse :: Color4 GLfloat
lightDiffuse = Color4 1.0 0.0 0.0 1.0

lightPosition :: Vertex4 GLfloat
lightPosition = Vertex4 1.0 1.0 1.0 0.0

initfn :: IO ()
initfn = let light0 = Light 0 
             light1 = Light 1
             light2 = Light 2
         in do diffuse light0 $= lightDiffuse
               position light0 $= lightPosition
               light light0 $= Enabled
               
               diffuse light1 $= lightDiffuse
               position light1 $= Vertex4 (-1.0) (-1.0) (-1.0) 0.0
               light light1 $= Enabled

               diffuse light2 $= lightDiffuse
               position light2 $= Vertex4 1.0 (-1.0) (-1.0) 0.0
               light light2 $= Enabled
         
               lighting $= Enabled

               depthFunc $= Just Lequal

               matrixMode $= Projection
               perspective 40.0 1.0 1.0 10.0
               matrixMode $= Modelview 0
               lookAt (Vertex3 0.0 0.0 5.0) (Vertex3 0.0 0.0 0.0) (Vector3 0.0 1.0 0.0)
  
               translate ((Vector3 0.0 0.0 (-1.0))::Vector3 GLfloat)
           --    rotate 60    ((Vector3 1.0 0.0 0.0)::Vector3 GLfloat)
           --    rotate (-20) ((Vector3 0.0 0.0 1.0)::Vector3 GLfloat)

cubewith :: MVar (FilterState, RateEstimate) -> IO ()
cubewith filterstate = do
  getArgsAndInitialize
  initialDisplayMode $= [DoubleBuffered, RGBMode, WithDepthBuffer]
  actionOnWindowClose $= MainLoopReturns
  createWindow "red 3D lighted cube"
  displayCallback $= display 
  reshapeCallback $= Just reshape
  
  displaystate <- newEmptyMVar
  putMVar displaystate qzero
  
  idleCallback $= Just (idle filterstate displaystate)
  initfn
  mainLoop

reshape s@(Size w h) = do
  viewport $= (Position 0 0, s)

idle :: MVar (FilterState, RateEstimate) -> MVar (Quat) -> IdleCallback
idle filterstate displaystate = do
  (fs, re) <- readMVar filterstate
  
  let qq = q fs

  rotate 2 ((Vector3 (-1.0) (-1.0) (-1.0))::Vector3 GLfloat)
  postRedisplay Nothing
