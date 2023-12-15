<template>
  <q-card flat class="q-py-lg q-px-sm">
    <q-stepper v-model="step" ref="stepper" color="primary" >
      <q-step
        :name="1"
        title="地图选择"
        icon="settings"
        :done="step > 1"
        :header-nav="step > 1"
      >
        <map-select :map-info="mapInfo" @update="update"></map-select>
        <q-stepper-navigation>
          <q-btn @click="startNavigation()" color="primary" label="Continue" :disable="mapInfo.id ? false : true"/>
        </q-stepper-navigation>
      </q-step>
        <q-step        
          :name="2"
          title="控制选择"
          icon="mdi-list-box-outline"
          :done="step > 2"
          :header-nav="step > 2"
        >
          <ControlSelect
            @back="backToStep1"
            :mapInfo="mapInfo"
          ></ControlSelect>
        </q-step>
    </q-stepper>
  </q-card>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted, ref } from 'vue';
import { MapInfo } from '@/components/models';
import { navStartReq, navPatrolReq, navEndReq,navStopReq } from '@/api/nav';
import ControlSelect from './ControlSelect.vue';
import { kinectStore } from '@/stores/kinectStore';
import MapSelect from '@/components/waypoint/MapSelect.vue';

let step = ref(1)

// 地图选择
let mapInfo = ref<MapInfo>({name:''})
function update(info:MapInfo) {

  mapInfo.value = info;
}

const kinect = kinectStore()

async function startNavigation() {
  step.value = 2;
  await navStartReq('get',<number>mapInfo.value.id);
  //kinect.setKinect(true)
}

async function endNavigation() {
  step.value = 1;
  navEndReq('get',{});
  //kinect.setKinect(false)
}

function backToStep1() {
  endNavigation()
}

function beforeunload() {
  if(step.value === 2 || step.value === 3) {

    navEndReq('get',{})
    kinect.setKinect(false)
  }
}

onMounted(() => {
  let tmpFunc = window.onbeforeunload
  if(tmpFunc !== null){  
      window.onbeforeunload = () => {
      beforeunload()
      tmpFunc()
    }
  } else {
    window.onbeforeunload = () => {
      beforeunload()
    }
  }
})

onUnmounted(()=> {
  beforeunload()
})
</script>
