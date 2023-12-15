<script>
import { defineComponent, h } from "vue";
export default defineComponent({
  setup:() => {
    const detailContent = h(
      'div',[
        h('div','不同指令需要包含不同关键字'),
        h('div','左转: 左转 右转:右转'),
        h('div','左: 左 右:右'),
        h('div','前: 前 后:后'),
        h('div','停止:停')
      ],
    )
    return () => detailContent
  }
  
});
</script>