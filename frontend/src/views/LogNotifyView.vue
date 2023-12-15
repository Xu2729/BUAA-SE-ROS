
<script lang="ts" setup >
import { useQuasar } from 'quasar';
import { logLatestReq } from '@/api/log';
import { onMounted,watch } from 'vue';
import { logStore } from '@/stores/logStore';
const $q = useQuasar()
let prev:null|object = null
const T = 10
const log = logStore()
let logInterval:NodeJS.Timeout|null = null
async function updateLog() {
  let response = await logLatestReq('get')
  if(response) {
    if(!prev) {
      prev = response.data.log
    }
    else  if (response.data.log.id !== prev.id)
      $q.notify({
        message:response.data.log.detail,
        caption: '新的日志,记得查看',
        type: 'warning',
        position:'right',
        timeout:T*800
      })
      prev = response.data.log
  }
}

watch(  () => log.show,
  (to, from) => {

    if(to) {
      logInterval = setInterval(updateLog,T*1000)
    } else {
      clearInterval(<NodeJS.Timeout>logInterval)
    }
  })

function init() {
  logInterval = setInterval(updateLog,T*1000)
}

onMounted(() => {
  if(log.show) {
    init()
  }
})

</script>
