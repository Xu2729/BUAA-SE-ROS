<template>
  <q-card class="q-pa-md">
    <q-splitter v-model="splitterModel" unit="px">
      <template v-slot:before>
        <q-tabs v-model="tab" vertical class="text-teal shadow-2" dense>
          <q-tab name="mapping" icon="mdi-map-legend" label="建图"/>
          <q-tab name="waypointMark" icon="mdi-map-marker-check-outline" label="航点标注"/>
        </q-tabs>
      </template>

      <template v-slot:after>
        <q-tab-panels v-model="tab" animated>
          <q-tab-panel name="mapping" style="display: flex">
            <mapping-tab></mapping-tab>
          </q-tab-panel>

          <q-tab-panel name="waypointMark" style="display: flex">
            <waypoint-mark-tab></waypoint-mark-tab>
          </q-tab-panel>

        </q-tab-panels>
      </template>
    </q-splitter>
  </q-card>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import MappingTab from '@/components/MappingTab.vue';
import WaypointMarkTab from '@/components/waypoint/WaypointMarkTab.vue';

let tab = ref('mapping');
let splitterModel = ref(70);
</script>

<style scoped></style>
