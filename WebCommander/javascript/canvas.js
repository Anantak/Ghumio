function handleSensorMsg(data) {
  var canvas = document.getElementById("main-display");
  var ctx = canvas.getContext("2d");
  ctx.strokeStyle = "blue";
  ctx.fillStyle = "blue";
  ctx.lineWidth = 1;
  ctx.font = "14px sans serif";

  ctx.clearRect(0, 0, 2000, 2000);

  var tags = data.april_msg;
  for (var i = 0; i < tags.tag_id.length; ++i) {
    var tag_id = tags.tag_id[i];
    var p1 = makePoint(tags.u_1[i], tags.v_1[i]);
    var p2 = makePoint(tags.u_2[i], tags.v_2[i]);
    var p3 = makePoint(tags.u_3[i], tags.v_3[i]);
    var p4 = makePoint(tags.u_4[i], tags.v_4[i]);
    var center = meanPoint([p1, p2, p3, p4]);

    ctx.fillText(tag_id.substr(9), center.x, center.y);
    drawPolygon(ctx, [p1, p2, p3, p4]);
  }
}

// Draws the given quad using the 2d context.
function drawPolygon(ctx, vertices) {
  if (vertices.length <= 1) return;
  for (var i = 0; i < vertices.length - 1; ++i) {
    drawLine(ctx, vertices[i], vertices[i+1])
  }
  drawLine(ctx, vertices[vertices.length - 1], vertices[0]);
}

// Draws the given line using the 2d context.
function drawLine(ctx, p1, p2) {
  ctx.beginPath();
  ctx.moveTo(p1.x, p1.y);
  ctx.lineTo(p2.x, p2.y);
  ctx.closePath();
  ctx.stroke();
}

// Returns mean point.
function meanPoint(points) {
  var xs = [], ys = [];
  for (var i in points) { xs.push(points[i].x); ys.push(points[i].y); }
  return makePoint(mean(xs), mean(ys));
}

// Returns a new point from the given x,y coords.
function makePoint(x, y) {
  return {x: x, y: y};
}

// Returns mean. Input should be array of numbers
function mean(values) {
  if (!values || !values.length) return 0;
  var sum = 0;
  for (var i in values) { sum += values[i]; }
  return sum / values.length;
}
