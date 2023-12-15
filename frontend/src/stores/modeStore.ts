import { defineStore } from 'pinia';
import { ref } from 'vue';

export const modeStore = defineStore(
  'mode',
  () => {
    const mode = ref(true);
    return { mode };
  },
  {
    persist: true,
  }
);
