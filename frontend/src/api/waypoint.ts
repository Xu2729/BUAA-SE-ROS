import { request, fileRequest } from './api';

export function mapListReq(method: string, params: object = {}) {
  const url = `/navigation/map_list`;
  return request(url, method, params);
}

export function pointListReq(method: string, query_id: number) {
  const url = `/navigation/point_list/${query_id}`;
  return request(url, method, {});
}

export function deletePointReq(method: string, query_id: number) {
  const url = `/navigation/delete/${query_id}`;
  return request(url, method, {});
}

export function renamePointReq(method: string, params: object) {
  const url = `/navigation/rename`;
  return request(url, method, params);
}

export function markPointReq(method: string, query_id: number, params: object) {
  const url = `/navigation/mark/${query_id}`;
  return request(url, method, params);
}
