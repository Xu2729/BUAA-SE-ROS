<template>
  <router-view />
  <kinect-display v-if="kinectStore().show"></kinect-display>
  <error-display v-if="userStore().isLogin && wsStore().ws"></error-display>
  <log-notify-view v-if="userStore().isLogin" ></log-notify-view>
</template>

<script setup lang="ts">
import KinectDisplay from './components/rosBridge/KinectDisplay.vue';
import { wsStore } from './stores/wsStore';
import { kinectStore } from './stores/kinectStore';
import { userStore } from './stores/userStore';
import "roslib/build/roslib"
import { rosFreeReq } from './api/ros';
import {  onMounted } from 'vue';
import errorDisplay from './components/rosBridge/errorDisplay.vue';
import LogNotifyView from './views/LogNotifyView.vue';
const ws = wsStore()
onMounted(() => {
  let tmpFunc = window.onbeforeunload
  if(tmpFunc !== null){  
      window.onbeforeunload = () => {
      ws.disconnectWS()
      rosFreeReq('get')
      tmpFunc()
    }
  } else {
    window.onbeforeunload = () => {
      ws.disconnectWS()
      rosFreeReq('get')
    }
  }
})
</script>

<style>
body.body--dark {
  background-color: black;
}

body.body--light {
  background-image: linear-gradient(to top, #cfd9df 0%, #e2ebf0 100%);
}

body {
}
</style>
