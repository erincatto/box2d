settings.outformat = "svg";

real px = 0.75bp;
unitsize(64px);

pen ink = rgb("231f20");
pen bodyPen = ink + 0.8px + miterjoin;
pen penA = rgb("a8322a") + 1.2px;
pen penB = rgb("1f5c99") + 1.2px;
pen guidePen = gray(0.45) + 0.6px + linetype(new real[] {4, 3});
pen linkPen = ink + 1.2px;
pen dimensionPen = ink + 0.8px;

real angleA = -8;
real angleB = 28;

pair anchorA = (0, 0);
pair anchorB = (2.6, 0.9);

pair halfA = (1.3, 0.5);
pair halfB = (0.9, 0.6);
pair localAnchorA = (0.8, 0.5);
pair localAnchorB = (-0.5, -0.4);

pair originA = anchorA - rotate(angleA) * localAnchorA;
pair originB = anchorB - rotate(angleB) * localAnchorB;

path boxA = shift(originA) * rotate(angleA) * box(-halfA, halfA);
path boxB = shift(originB) * rotate(angleB) * box(-halfB, halfB);

filldraw(boxA, palegreen, bodyPen);
filldraw(boxB, lightgray, bodyPen);

void origin(pair p)
{
  real r = 0.07;
  draw((p.x - r, p.y) -- (p.x + r, p.y), ink + 0.8px);
  draw((p.x, p.y - r) -- (p.x, p.y + r), ink + 0.8px);
}

draw(originA -- anchorA, guidePen);
draw(originB -- anchorB, guidePen);
origin(originA);
origin(originB);

label("$A$", originA + rotate(angleA) * (-0.35, 0), ink);
label("$B$", originB + rotate(angleB) * (0.35, 0), ink);

draw(anchorA -- anchorB, linkPen);

dot(anchorA, penA + 5px);
dot(anchorB, penB + 5px);
label("$p_A$", anchorA, NW, penA);
label("$p_B$", anchorB, NW, penB);

pair along = unit(anchorB - anchorA);
pair normal = rotate(90) * along;
pair offset = -0.55 * normal;
pair startDim = anchorA + offset;
pair endDim = anchorB + offset;
draw(anchorA -- startDim, guidePen);
draw(anchorB -- endDim, guidePen);
draw(startDim -- endDim, dimensionPen, Arrows(5px));
label("$L$", (startDim + endDim) / 2, -normal, ink);
