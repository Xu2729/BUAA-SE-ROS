import { defineStore } from 'pinia';
import { ref, computed } from 'vue';
import 'roslib/build/roslib';
import ROSLIB from 'roslib';
export const kinectStore = defineStore('kinect', () => {
  const show = ref(false);
  function setKinect(value: boolean) {
    show.value = value;
  }
  return { show, setKinect };
});
