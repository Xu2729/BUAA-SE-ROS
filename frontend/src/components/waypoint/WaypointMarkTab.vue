<template>
  <div class="q-pa-md">
    <q-stepper
      v-model="step"
      header-nav
      ref="stepper"
      color="primary"
      animated
    >
      <q-step
        :name="1"
        title="地图选择"
        icon="settings"
        :done="step > 1" 
        :header-nav="step > 1"
      >
        <map-select :map-info="mapInfo" @update="update" :key="flag"></map-select>

        <q-stepper-navigation class="q-gutter-x-md">
          <q-btn @click="deleteMap" color="red" label="删除地图" :disable="mapInfo.id ? false : true"/>
          <q-btn @click="() => { step = 2 }" color="primary" label="Continue" :disable="mapInfo.id ? false : true"/>
        </q-stepper-navigation>
      </q-step>

      <q-step
        :name="2"
        title="航点操作"
        icon="mdi-pencil-outline"
        :done="step > 2"
        :header-nav="step > 2"
      >
        <waypoint-edit :map-info="mapInfo" :editable="true"></waypoint-edit>
        <q-stepper-navigation>
          <q-btn @click="step = 1" color="primary" label="保存结果" />
          <q-btn flat @click="step = 1" color="primary" label="Back" class="q-ml-sm" />
        </q-stepper-navigation>
      </q-step>

    </q-stepper>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue';
import MapSelect from './MapSelect.vue';
import {navMapDeleteReq} from '@/api/nav'
import { MapInfo } from '../models';
import WaypointEdit from './WaypointEdit.vue';
import { useRouter } from 'vue-router';


let step = ref(1)
let mapInfo = ref<MapInfo>({name:''})

function update(info:MapInfo) {
  console.log(info);
  mapInfo.value = info
}
const router = useRouter()
const flag = ref(0)
function save() {
  return 
}

function deleteMap() {
  navMapDeleteReq('delete',mapInfo.value.id)
  flag.value ^= 1
  mapInfo.value = {name:''}
}

</script>