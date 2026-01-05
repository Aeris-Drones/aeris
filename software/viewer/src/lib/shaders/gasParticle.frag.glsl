precision mediump float;

uniform vec3 uColorLow;
uniform vec3 uColorMid;
uniform vec3 uColorHigh;

varying float vAge;
varying float vConcentration;

void main() {
  vec2 coord = gl_PointCoord - vec2(0.5);
  float dist = length(coord);
  if (dist > 0.5) discard;

  float alpha = 1.0 - smoothstep(0.2, 0.5, dist);

  float fadeIn = smoothstep(0.0, 0.1, vAge);
  float fadeOut = 1.0 - smoothstep(0.85, 1.0, vAge);
  alpha *= fadeIn * fadeOut;

  vec3 color;
  if (vConcentration < 0.5) {
    color = mix(uColorLow, uColorMid, vConcentration * 2.0);
  } else {
    color = mix(uColorMid, uColorHigh, (vConcentration - 0.5) * 2.0);
  }

  gl_FragColor = vec4(color, alpha * 0.5);
}
