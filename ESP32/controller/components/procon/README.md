# components/procon
`my_platform.c`のコールバック群は，`app_main()`タスクで呼ばれるループ処理`btstack_run_loop_execute()`で呼ばれる．そのため，sdkconfigで決めることができる`CONFIG_ESP_MAIN_TASK_STACK_SIZE`は，
Bluetooth接続時などは，スタックが深くなり，デフォルトの3584だと足りなくなることがある．
そのため,`on_device_connected()`などが呼ばれるときのスタックの深さを，
```C
ESP_LOGI("STACK", "main task stack remaining (words): %u", uxTaskGetStackHighWaterMark(NULL));
```
で確認し，適切な大きさを設定すると良い．

もしくは，

```C
void bt_run_loop_task(void *arg){
    btstack_run_loop_execute(); // ここでnever return
}

void app_main(void){
    // ... 既存の初期化処理(arms_init(), can_init_and_start(), 各種xTaskCreateなど)は全部先にやっておく ...

    xTaskCreatePinnedToCore(
        bt_run_loop_task,
        "bt_run_loop",
        8192, // ここで明示的にサイズを指定。CONFIG_ESP_MAIN_TASK_STACK_SIZEとは独立
        NULL,
        1, // app main プライオリティ(ESP_TASK_MAIN_PRIO)はデフォルトでは1
        NULL,
        APP_CPU_NUM
    );

    // app_main()はここでreturnできる
}
```
として，sdkconfigの設定と切り離してコード内で管理できるようにすると良い．
このとき，`app_main()`はreturnすると割り当てられていたスタック領域(`CONFIG_ESP_MAIN_TASK_STACK_SIZE`)をヒープに返却する．