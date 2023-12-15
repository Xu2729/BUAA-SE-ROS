const int2roleMap = {
  0: 'user',
  1: 'admin',
};

export function int2Role(x: 0 | 1): string {
  return int2roleMap[x];
}
