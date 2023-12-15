<template>
    <q-card flat class="q-py-lg q-px-sm">
      <auto-component
      :has-save="false"
      :tip-brief="false"
        @launch="startCtrl"
        @prev="backToSelect"
      >
        <template v-slot:map_display>
          <nav-display></nav-display>
        </template>
      </auto-component>
  </q-card>
</template>

<script setup lang="ts">
import AutoComponent from '@/components/AutoComponent.vue';
import NavDisplay from '@/components/rosBridge/NavDisplay.vue';
import { ref, defineEmits } from 'vue';
import { navStartReq } from '@/api/nav'
let step = ref(1)
const emits = defineEmits(['back'])
// 用户控制

function startCtrl() {
  return
}

function backToSelect() {
  emits('back')
}
</script>
