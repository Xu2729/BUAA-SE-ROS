<template>
  <div>
    <q-table
      title="人员图像管理"
      :rows="faceList"
      :columns="columns"
      row-key="name"
      selection="multiple"
      grid
      hide-header
    >
      <template v-slot:top-right>
        <q-btn color="primary" round flat icon="mdi-plus-circle" size="lg" @click="imgDialog = true">
        </q-btn>
      </template>

      <template v-slot:item="props">
        <div
          class="q-pa-xs"
          :style="props.selected ? 'transform: scale(0.95);' : ''"
        >
          <q-card :class="props.selected ? 'bg-grey-2' : ''" min-width="100px">
            <q-card-section>
              <q-btn round color="red" flat @click="deleteImg(props.row.id)" icon="mdi-trash-can-outline"></q-btn>
            </q-card-section>
            <q-separator />
            <q-list dense>
              <q-item>
                <q-item-section>
                  <q-item-label>姓名</q-item-label>
                </q-item-section>
                <q-item-section>
                  <q-item-label>{{ props.row.name }} </q-item-label>
                </q-item-section>
              </q-item>
              <q-item>
                <q-item-section side>
                  <q-avatar size="150px" square>
                    <img :src="props.row.url"/>
                  </q-avatar>
                </q-item-section>
              </q-item>
            </q-list>
          </q-card>
        </div>
      </template>
      <template v-slot:no-data="{}">
        <div class="full-width row flex-center text-accent q-gutter-sm">
          <q-icon size="2em" name="sentiment_dissatisfied" />
          <span>
            还没有人员照片，记得上传图像
          </span>
        </div>
      </template>
    </q-table>
    <q-dialog v-model:model-value="imgDialog" min-width="200px">
      <q-card class="q-pa-md">
          <div class="q-gutter-y-md column" style="min-width: 300px">
            <q-avatar class="q-mt-xs avatar" size="150px" square>
              <img :src="imgPreview" />
            </q-avatar>

            <q-file v-model="file" dense accept=".jpg" max-file-size="128000" />

            <q-input
              label="输入人员姓名"
              v-model:model-value="name"
            ></q-input>

            <div
              style="
                display: flex;
                flex-direction: row;
                justify-content: space-between;
              "
            >
              <q-btn
                flat
                dense
                class="text-error"
                @click="imgDialog = false"
              >
                取消
              </q-btn>
              <q-btn flat dense class="text-primary" @click="submitImg" :disable="file === null || name === ''">
                确认
              </q-btn>
            </div>
          </div>
        </q-card>
    </q-dialog>
  </div>
</template>

<script lang="ts" setup>
import {faceUplaodReq, faceDeleteReq, faceListReq} from "@/api/face"
import { ref, onMounted, computed } from 'vue'

const faceList = ref([])
const columns = [{
    name: 'name',
    label: '姓名',
    field: 'name'
  },
  {
    name: 'img',
    label: '图片',
    field: 'url'
  }

]

const imgDialog = ref(false)
const file = ref<null|File>(null)
const name = ref('')

let imgPreview = computed(() => {
  const windowURL = window.URL || window.webkitURL;
  const ret = !(file.value instanceof File)
    ? 'https://gimg2.baidu.com/image_search/src=http%3A%2F%2Fimg2.doubanio.com%2Fview%2Fgroup_topic%2Fl%2Fpublic%2Fp515017572.jpg&refer=http%3A%2F%2Fimg2.doubanio.com&app=2002&size=f9999,10000&q=a80&n=0&g=0n&fmt=auto?sec=1684653688&t=77fc1a857d099ee9c9331a633125f65f'
    : windowURL.createObjectURL(file.value);
  return ret;
});

async function deleteImg(id:number) {
  await faceDeleteReq('delete',id)
  init()
}

async function submitImg() {
  await faceUplaodReq('post',{file:file.value}, name.value)
  imgDialog.value = false
  file.value = null
  name.value = ''
  init()
}

async function init() {
  let response = await faceListReq('get')
  faceList.value = response.data.faces
}

onMounted(() => {
  init()
})

</script>


<style scoped>
.avatar {
  left: 50%;
  transform: translateX(-50%);
  display: block;
}
</style>