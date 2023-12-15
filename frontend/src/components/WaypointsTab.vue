<template>
  <q-card flat style="min-width: 100px">
    <div flat style="display: flex; flex-direction: row">
      <div class="q-pr-sm">
        <q-card-section>
          <q-img src="@/assets/cursor.svg" width="100px" height="100px"></q-img>
        </q-card-section>
        <div>
          <div style="text-align: center"><strong>点击标注</strong></div>
        </div>
        <div style="text-align: center">
          <q-btn
            :icon="listen ? 'mdi-pause' : 'mdi-play'"
            color="secondary"
            @click="handleInput()"
            :label="listen ? 'end' : 'begin'"
          />
        </div>
      </div>
      <q-separator vertical inset size="2px" />
      <div class="q-pl-sm">
        <q-img
          src="https://cdn.quasar.dev/img/parallax2.jpg"
          :width="imgWidth"
          :height="imgheight"
          fit="scale-down"
        />
      </div>
    </div>
  </q-card>
</template>

<script setup lang="ts">
import { computed, ref } from 'vue';
import { useQuasar } from 'quasar';
const $q = useQuasar();
let imgWidth = computed(() => ($q.screen.width * 2) / 3 + 'px');
let imgheight = computed(() => ($q.screen.height * 3) / 4 + 'px');
// 键盘输入
let listen = ref(false);
function handleInput() {
  listen.value = !listen.value;
}
</script>
