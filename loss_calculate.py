import openpyxl
from openpyxl import Workbook
from evaluate import read_results_from_excel, write_results_to_excel, write_loss_to_excel


if __name__ == "__main__":
    UAV_NUM = [50, 125, 200]
    DAMAGE_RATIO = [0, 0.1, 0.2, 0.3, 0.4, 0.5]
    excel_path = "/home/yjq/ProjectGroup6/src/result.xlsx"

    for uav_num in UAV_NUM:
        total_loss_list = []
        result_list = []
        o1_result0, d1_result0, a1_result0 = read_results_from_excel(uav_num, DAMAGE_RATIO[0], filename=excel_path)
        for damage_datio in DAMAGE_RATIO:
            o1_result, d1_result, a1_result = read_results_from_excel(uav_num, damage_datio, filename=excel_path)
            obeserve_loss = (o1_result0 - o1_result) / o1_result0
            decision_loss = (d1_result0 - d1_result) / d1_result0
            attack_loss = (a1_result0 - a1_result) / a1_result0
            total_loss = (obeserve_loss + decision_loss + attack_loss) / 3
            total_loss_list.append(total_loss)

        for i in range(len(total_loss_list) - 1):
            result = total_loss_list[i + 1] / DAMAGE_RATIO[i + 1]
            result_list.append(result)
            write_loss_to_excel(uav_num, DAMAGE_RATIO[i + 1], result, filename=excel_path)
        print("result_list", result_list)
