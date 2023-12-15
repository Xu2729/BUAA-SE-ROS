<template>
  <q-stepper v-model="step" ref="stepper" color="primary">
    <q-step        
      :name="1"
      title="航点排序"
      icon="mdi-list-box-outline"
      :done="step > 2"
      :header-nav="step > 2"
      style="padding: 0%!important;"
    >
    <waypoint-sort
      :editable="false"
      :map-info="mapInfo"
      @next="startPatrol"
      @prev="emits('back')"
    ></waypoint-sort>
    </q-step>
    <q-step
      :name="2"
      title="值班模式"
      icon="mdi-robot"
      :done="step > 2"
      :header-nav="step > 2"
      style="padding: 0%!important;"
    >
      <navigation-ctrl
        @back-to-step2="backToStep1"
        :path="patrolInfo?.path"
        :loop="patrolInfo?.loop"
      ></navigation-ctrl>
    </q-step>
  </q-stepper>
</template>

<script setup lang="ts">
import { onMounted, onUnmounted, ref, defineEmits } from 'vue';
import NavigationCtrl from '@/components/NavigationCtrl.vue';
import { MapInfo } from '@/components/models';
import { navStopReq } from '@/api/nav';
import WaypointSort from '@/components/waypoint/WaypointSort.vue';
interface autoProps {
  mapInfo: MapInfo;
}

const props = withDefaults(defineProps<autoProps>(), {
  mapInfo: {},
});

const emits = defineEmits(['back'])
let step = ref(1)

// 航点排序
let patrolInfo = ref<{path:{x:number,y:number,yaw:number,name:string}[],loop:number}|null>(null)
function startPatrol(info:{path:{x:number,y:number,yaw:number,name:string}[],loop:number}) {
  patrolInfo.value = info;
  if (patrolInfo.value.loop != 0) {
      patrolInfo.value.path.push(patrolInfo.value.path[0])
  }
  step.value = 2;
}


// 值班巡逻
function backToStep1() {
  step.value = 1
}

</script>
