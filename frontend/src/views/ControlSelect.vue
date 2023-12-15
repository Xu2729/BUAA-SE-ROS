<template>
  <q-splitter v-model="splitterModel" unit="px">
    <template v-slot:before>
      <q-tabs v-model="tab" vertical class="text-teal shadow-2" dense>
        <q-tab 
          name="navigation"
          icon="mdi-shield-sun-outline"
          label="值班模式"
        />
        <q-tab
          name="autonomy"
          icon="mdi-map-marker-check-outline"
          label="用户控制"
          
        />
      </q-tabs>
    </template>

    <template v-slot:after>
      <q-tab-panels v-model="tab" animated>
        <q-tab-panel name="navigation" style="display: flex">
          <NavigationView
            @back="emits('back')"
            :mapInfo="mapInfo"
          ></NavigationView>
        </q-tab-panel>

        <q-tab-panel name="autonomy" style="display: flex">
          <AutonomyView
            @back="emits('back')"
          ></AutonomyView>
        </q-tab-panel>
      </q-tab-panels>
    </template>
  </q-splitter>
</template>

<script setup lang="ts">
import { ref,defineEmits, defineProps } from 'vue';
import AutonomyView from './AutonomyView.vue';
import NavigationView from './NavigationView.vue';
import { MapInfo } from '@/components/models';
interface autoProps {
  mapInfo: MapInfo;
}

const props = withDefaults(defineProps<autoProps>(), {
  mapInfo: {},
});

const tab = ref('navigation')

const splitterModel = ref(70);
const emits = defineEmits(['back'])
</script>