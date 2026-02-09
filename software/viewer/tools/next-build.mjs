import { spawnSync } from 'node:child_process';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const suppressWarningPath = path.join(__dirname, 'suppress-baseline-warning.cjs');
const nodeOptions = process.env.NODE_OPTIONS ? `${process.env.NODE_OPTIONS} ` : '';

const env = {
  ...process.env,
  NODE_OPTIONS: `${nodeOptions}--require ${suppressWarningPath}`,
};

const result = spawnSync('next', ['build', '--webpack'], {
  stdio: 'inherit',
  shell: process.platform === 'win32',
  env,
});

process.exit(result.status ?? 1);

