export interface Todo {
  id: number;
  content: string;
}

export interface Meta {
  totalCount: number;
}

export interface MapInfo {
  id?: number;
  url?: string;
  name?: string;
  x: number;
  y: number;
}

export interface PointInfo {
  x?: number;
  y?: number;
  theta?: number;
}
