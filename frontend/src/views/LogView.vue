<template>
  <div style="display: flex; flex-direction: row;min-width:500px">
  <q-list>
    <q-separator spaced inset />
    <template v-for="log,i in partOfLog" :key="log.id">
      <q-item @click="clickItem(log)" clickable>
        <q-item-section>
          <q-item-label>{{ log.detail }}</q-item-label>
          <q-item-label caption>{{ log.time }}</q-item-label>
        </q-item-section>
      </q-item>
      <q-separator spaced inset  v-if="i != log.length"/>
    </template>

      <q-item >
        <q-item-section >
          <q-item-label
          style="
                display: flex;
                flex-direction: row;
                justify-content:center
              "
          >
          <q-pagination
            v-if="pageOfLog > 1"
            v-model="cur"
            :max="pageOfLog"
            direction-links
            flat
            color="grey"
            active-color="primary"
          />
        </q-item-label>
      </q-item-section>
      </q-item>
  </q-list>

  <q-separator vertical></q-separator>
  <div style="witdh:500px" v-if="chosenOne">
    <q-list>
    <q-item>
      <q-item-section header>
        <q-item-label>
          日志详情
        </q-item-label>
      </q-item-section>
    </q-item>
    <q-item>
      <q-item-section>
        <q-item-label>
          日志内容:
        </q-item-label>
      </q-item-section>
      <q-item-section>
        <q-item-label side>
          {{ chosenOne.detail }}
        </q-item-label>
      </q-item-section>
    </q-item>
    <q-item>
      <q-item-section>
        <q-item-label>
          时间
        </q-item-label>
      </q-item-section>
      <q-item-section>
        <q-item-label side>
          {{ chosenOne.time }}
        </q-item-label>
      </q-item-section>
    </q-item>
    <q-item>
      <q-img :src="chosenOne.url" width="450px" height="450px" fit="contain">
      </q-img>
    </q-item>
    </q-list>
  </div>
</div>
</template>

<script lang="ts" setup >
import {ref, onMounted, computed} from 'vue'
import {logListReq} from '@/api/log'

const logs = ref([])
const cur = ref(1)
const size = 7
const chosenOne = ref<unknown>(null)

const partOfLog = computed(() => {
  return logs.value.slice((cur.value-1)*size,cur.value*size)
})

const pageOfLog = computed(() => {
  return Math.ceil(logs.value.length/size)
})

async function init() {
  let response = await logListReq('get')
  logs.value = response.data.logs
}

function clickItem(log:object) {
  chosenOne.value = log
}

onMounted(() => {
  init()
})

</script>
