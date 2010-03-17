import Graphics.Rendering.OpenGL
import Graphics.UI.GLUT
import Cube

main = do
  (progname, _) <- getArgsAndInitialize
  createWindow "Hello World"
  displayCallback $= display
  reshapeCallback $= Just reshape
  keyboardMouseCallback $= Just keyboardMouse
  mainLoop

reshape s@(Size w h) = do
  viewport $= (Position 0 0, s)

display = do
  clear [ColorBuffer]
  cube (0.2::GLfloat)
  flush

keyboardMouse key state modifiers position = return ()
