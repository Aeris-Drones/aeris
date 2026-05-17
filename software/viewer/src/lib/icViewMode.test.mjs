import test from "node:test";
import assert from "node:assert/strict";

import { isIcViewModeQueryValue } from "./icViewMode.js";

test("isIcViewModeQueryValue accepts the supported URL activation values", () => {
  assert.equal(isIcViewModeQueryValue("1"), true);
  assert.equal(isIcViewModeQueryValue("true"), true);
  assert.equal(isIcViewModeQueryValue("IC"), true);
});

test("isIcViewModeQueryValue rejects operator-mode and missing values", () => {
  assert.equal(isIcViewModeQueryValue(null), false);
  assert.equal(isIcViewModeQueryValue("0"), false);
  assert.equal(isIcViewModeQueryValue("operator"), false);
});

