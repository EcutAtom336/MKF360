CRYSTAL_FREQ = 8000000
PDM_CLK = 2048000

plan_cnt = 0

for divm in range(1, 63):
    divm_out = CRYSTAL_FREQ / divm
    for divn in range(4, 512):
        divn_out = divm_out * divn
        if divn_out < 150000000 or divn_out > 960000000:
            continue
        for divp in range(1, 128):
            divp_out = divn_out / divp
            if divp_out % PDM_CLK == 0 and divp_out < 150000000:
                print(divm, divn, divp)
                plan_cnt += 1
            if plan_cnt > 10:
                exit(0)
