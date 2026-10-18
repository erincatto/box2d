settings.outformat = "svg";

real px = 0.75bp;
unitsize(64px);

pen ink = rgb("231f20");
pen bodyPen = ink + 0.8px + miterjoin;
pen penA = rgb("a8322a") + 1.2px;
pen penB = rgb("1f5c99") + 1.2px;
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});
pen axisPen = gray(0.35) + 0.8px + linetype(new real[] {6, 4});
pen dimensionPen = ink + 0.8px;

real axisLength = 0.8;
real arrowSize = 6px;

real axisAngle = 20;
pair axis = dir(axisAngle);
pair normal = dir(axisAngle + 90);

real translation = 2.6;

pair frameA = (0, 0);
pair frameB = frameA + translation * axis;

pair halfA = (1.3, 0.5);
pair halfB = (0.95, 0.7);
pair localFrameA = (1.0, 0.15);
pair localFrameB = (-0.65, -0.28);

pair originA = frameA - rotate(axisAngle) * localFrameA;
pair originB = frameB - rotate(axisAngle) * localFrameB;

path boxA = shift(originA) * rotate(axisAngle) * box(-halfA, halfA);
path boxB = shift(originB) * rotate(axisAngle) * box(-halfB, halfB);

filldraw(boxA, palegreen, bodyPen);
filldraw(boxB, lightgray, bodyPen);

draw(frameA - 1.2 * axis -- frameB + 1.8 * axis, axisPen);

void origin(pair p)
{
  real r = 0.07;
  draw((p.x - r, p.y) -- (p.x + r, p.y), ink + 0.8px);
  draw((p.x, p.y - r) -- (p.x, p.y + r), ink + 0.8px);
}

draw(originA -- frameA, guidePen);
draw(originB -- frameB, guidePen);
origin(originA);
origin(originB);

label("$A$", originA + rotate(axisAngle) * (-0.35, 0), ink);
label("$B$", originB + rotate(axisAngle) * (0.35, 0), ink);

void frame(pair p, pen axisPen, string xs, string ys)
{
  draw(p -- p + axisLength * axis, axisPen, Arrow(arrowSize));
  draw(p -- p + 0.6 * axisLength * normal, axisPen, Arrow(arrowSize));
  label(xs, p + axisLength * (axis - 0.2 * normal), axis, axisPen);
  label(ys, p + 0.6 * axisLength * normal, normal, axisPen);
}

frame(frameA, penA, "$x_A$", "$y_A$");
frame(frameB, penB, "$x_B$", "$y_B$");

dot(frameA, ink + 4px);
dot(frameB, ink + 4px);

pair offset = -0.95 * normal;
pair startDim = frameA + offset;
pair endDim = frameB + offset;
draw(frameA -- startDim, guidePen);
draw(frameB -- endDim, guidePen);
draw(startDim -- endDim, dimensionPen, Arrows(5px));
label("$d$", (startDim + endDim) / 2, -normal, ink);
