import { defineStore } from 'pinia';
import { ref, computed } from 'vue';
import 'roslib/build/roslib';
import ROSLIB from 'roslib';
export const wsStore = defineStore('ws', () => {
  const ws = ref<ROSLIB.Ros | null>(null);
  function connectWS(ros: ROSLIB.Ros) {
    ws.value = ros;
  }
  function disconnectWS() {
    ws.value = null;
  }
  return { connectWS, disconnectWS, ws };
});
