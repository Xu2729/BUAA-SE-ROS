<template>
  <MapCtrl :has-save="true" @save="save" @cancel="cancel" @launch="launch">
    <template v-slot:map_display>
      <MapDisplay></MapDisplay>
    </template>
  </MapCtrl>
</template>

<script setup lang="ts">
import { mapSaveReq, mapCancelReq, mapLaunchReq } from '@/api/mapping';
import MapDisplay from './rosBridge/MapDisplay.vue';
import MapCtrl from './MapCtrl.vue';

async function save(name:string) {
  let response = await mapSaveReq('post',{name:name})
  if(response) {
    setTimeout(() => {cancel()},8000)
  }
}

function launch() {
  mapLaunchReq('get',{})
}

function cancel() {
  mapCancelReq('get',{})
}

</script>

<style scoped lang="scss">

</style>
