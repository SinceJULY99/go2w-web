// Geometry shared by the map view and its coordinate regression checks.
export function rotatePoint(point, angle){
  const cosine = Math.cos(angle), sine = Math.sin(angle);
  return { x:cosine*point.x - sine*point.y, y:sine*point.x + cosine*point.y };
}

export function wallAngle(points, target){
  const [a, b] = points;
  const heading = Math.atan2(b.y-a.y, b.x-a.x);
  const desired = target === 'vertical' ? Math.PI/2 : 0;
  // A wall is an unoriented line. Reversing the selected points keeps the result.
  const difference = desired - heading;
  return ((difference + Math.PI/2) % Math.PI + Math.PI) % Math.PI - Math.PI/2;
}

export function screenToWorld(px, py, width, height, view, angle){
  const offset = rotatePoint({ x:(px-width/2)/view.s, y:-(py-height/2)/view.s }, -angle);
  return { x:view.cx+offset.x, y:view.cy+offset.y };
}
