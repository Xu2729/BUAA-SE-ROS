<template>
  <q-card flat class="q-py-lg q-px-sm">
    <q-splitter v-model="splitterModel" unit="px">
      <template v-slot:before>
        <q-tabs v-model="tab" vertical class="text-teal shadow-2" dense>
          <q-tab 
            name="staff"
            icon="mdi-office-building-cog"
            label="人员管理"
          />
          <q-tab
            name="log"
            icon="mdi-alert-box-outline"
            label="险情日志"
            
          />
        </q-tabs>
      </template>

      <template v-slot:after>
        <q-tab-panels v-model="tab" animated>
          <q-tab-panel name="staff" style="display: flex">
            <StaffView></StaffView>
          </q-tab-panel>

          <q-tab-panel name="log" style="display: flex">
            <LogView></LogView>
          </q-tab-panel>
        </q-tab-panels>
      </template>
    </q-splitter>
  </q-card>
</template>

<script setup lang="ts">
import { ref } from 'vue';
import StaffView from "./StaffView.vue"
import LogView from './LogView.vue';

const tab = ref('staff')

const splitterModel = ref(70);
</script>