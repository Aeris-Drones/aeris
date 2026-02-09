const originalWarn = console.warn.bind(console);

console.warn = (...args) => {
  const first = args[0];
  if (
    typeof first === 'string' &&
    first.startsWith('[baseline-browser-mapping] The data in this module is over two months old.')
  ) {
    return;
  }
  return originalWarn(...args);
};

