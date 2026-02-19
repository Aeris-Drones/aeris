export const KNOWN_MODALITIES = new Set(["thermal", "acoustic", "gas"]);

export function normalizeModalities(rawModalities) {
  if (!Array.isArray(rawModalities)) {
    return [];
  }

  const normalized = rawModalities
    .map((modality) => (typeof modality === "string" ? modality.trim().toLowerCase() : ""))
    .filter((modality) => KNOWN_MODALITIES.has(modality));

  return Array.from(new Set(normalized));
}
