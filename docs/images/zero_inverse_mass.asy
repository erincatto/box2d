settings.outformat = "svg";
settings.render = 0;

real px = 0.75bp;

pen ink = rgb("231f20");
pen bodyFill = rgb("d7e3f4");
pen groundFill = gray(0.75);
pen bodyPen = ink + 1.2px;
pen goodPen = rgb("1f5c99") + 1.6px;
pen badPen = rgb("a8322a") + 1.6px;
pen guidePen = gray(0.5) + 0.9px + linetype(new real[] {4, 3});
pen text = fontsize(12pt);

unitsize(150px);
real arrowSize = 8px;
real halfWidth = 1.15;
real halfHeight = 0.5;

// The body pivot sits at r from the center of mass and the target on the ground at w along the x axis.
// With w different from r no rotation can bring the two together.
real r = 0.8;
real w = 1.25 * r;
real theta = 50;

path body = rotate(theta) * box((-halfWidth, -halfHeight), (halfWidth, halfHeight));
filldraw(body, bodyFill, bodyPen);

draw(circle((0, 0), r), guidePen);
draw((0, 0) -- (w + 0.2, 0), guidePen);

pair b = r * dir(theta);
pair a = (w, 0);

real pivotRadius = 0.045;
real targetHalf = 0.045;

// Stop the arrows at the edge of the pivot markers so the heads stay visible
draw((0, 0) -- b - pivotRadius * dir(theta), goodPen, Arrow(arrowSize));
label("$\mathbf{r}$", 0.55 * b, rotate(90) * dir(theta), goodPen + text);

draw(arc((0, 0), 0.28, 0, theta), ink + 0.8px);
label("$\theta$", 0.28 * dir(0.5 * theta), dir(0.5 * theta), ink + text);

// Distance from the center of mass to the target, drawn below the guide line
real dimY = -0.12;
draw((0, dimY) -- (w, dimY), ink + 0.8px, Arrows(0.8 * arrowSize));
label("$w$", (0.35 * w, dimY), S, ink + text);

pair e = unit(a - b);
draw(b + pivotRadius * e -- a - 1.6 * targetHalf * e, badPen, Arrow(arrowSize));
label("$\mathbf{d}$", 0.5 * (a + b) + (-0.02, 0.02), unit(rotate(90) * (a - b)), badPen + text);

filldraw(box(a - (targetHalf, targetHalf), a + (targetHalf, targetHalf)), groundFill, ink + 0.9px);
label("$\mathbf{p}_A$", a + (targetHalf, -targetHalf), SE, ink + text);
filldraw(circle(b, pivotRadius), white, ink + 1.2px);
label("$\mathbf{p}_B$", b + pivotRadius * dir(theta), dir(theta), ink + text);

// The usual center of mass symbol, a circle with two filled quadrants
real radius = 0.07;
fill((0, 0) -- arc((0, 0), radius, theta, theta + 90) -- cycle, ink);
fill((0, 0) -- arc((0, 0), radius, theta + 180, theta + 270) -- cycle, ink);
draw(circle((0, 0), radius), ink + 0.9px);
label("$m^{-1} = 0$", (-radius - 0.04, 0), W, ink + text);
