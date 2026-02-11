/**
 * Type declarations for GLSL shader file imports.
 *
 * Allows importing .glsl files as strings for use with Three.js shaders.
 * Webpack/Vite loaders handle the actual file transformation.
 *
 * @example
 * import vertexShader from './shaders/mesh.vert.glsl';
 * const material = new THREE.ShaderMaterial({ vertexShader, ... });
 */
declare module '*.glsl' {
  const content: string;
  export default content;
}

/** Fragment shader files (.frag, .fragment.glsl) */
declare module '*.frag' {
  const content: string;
  export default content;
}

/** Vertex shader files (.vert, .vertex.glsl) */
declare module '*.vert' {
  const content: string;
  export default content;
}
