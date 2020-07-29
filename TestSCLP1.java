package packing.Test;

import packing.Data.*;
import packing.Dynamic.Baseline;
import packing.Dynamic.Dynamic;
import packing.IO.InstanceLoader;
import packing.Common.*;
import packing.IO.InstanceLoader2;
import packing.SCLP.*;

import java.io.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Random;

public class TestSCLP {
    public static void main(String[] args) throws IOException {
        //  /root/Desktop/njubpp/
        BufferedWriter out = new BufferedWriter(new FileWriter("result/result.txt"));
        File f = new File("data");
        /*BufferedWriter out = new BufferedWriter(new FileWriter("result/result.txt"));
        File f = new File("data");*/
        File[] fs1 = f.listFiles();
        for (int fl = 0; fl < fs1.length; fl++) {
            String path = fs1[fl].getAbsolutePath();
            String name = fs1[fl].getName();
            System.out.println("Current file >>> " + name + " start to load instance>>>...");

            Cuboid container = new Cuboid(589,235,239); // Unified vehicle

            Instance[] insts = InstanceLoader2.load_inst(path);
            System.out.println("load instances successfully>>>...");

            int pb = 4; // the number of blocks placed by worker
            double ts_rate_sum = 0;
            double[] ts_p_rate_sum = new double[pb];
            double[] bl_rate_sum = new double[pb];
            double[] dy_rate_sum = new double[pb];
            int[] placed_id = new int[pb];

            for(int i = 0; i < insts.length; i++){
                ArrayList<Solution> off_sols = new ArrayList<>();
                System.out.println("Instance>>>>" + i);
                Instance inst = insts[i];
                inst.bin = container;
                BlockGeneration bg = new BlockGeneration(inst);
                ArrayList<Block> bks = bg.generate_blocks(100, 100, 100, 10000, 0.99);
                System.out.println("Block Number>>>" + bks.size());
                TreeSearch ts = new TreeSearch(inst);
                System.out.println("start to tree search>>>...");
                Solution s = ts.solve(bks, 1, 5, 2);
                off_sols.add(s);
                double ts_rate = ((double)s.vol) / inst.bin.v;
                ts_rate_sum += ts_rate;
                System.out.println("Tree search loading rate>>> " + ((double)s.vol) / inst.bin.v);
                System.out.println("The number of off-line solutions: " + off_sols.size());

                Solution init_sol = new Solution(inst);
                ArrayList<Space> init_sps = new ArrayList<>();
                init_sps.add(new Space(inst));
                Node n = new Node(inst, init_sol, init_sps, bks);

                for (int j = 0; j < pb; j++) {
                    if (!n.bks.isEmpty() && !n.sps.isEmpty()) {
                        Collections.sort(n.bks, Comparator.comparingLong(bk -> bk.vol));
                        Block _bk = n.bks.get(0);
                        for (int k = 0; k < n.sps.size(); k++) {
                            Space _sp = n.sps.get(k);
                            if (_sp.contains(_bk)) {
                                placed_id[j] = _bk.id;
                                n.add_block(_sp, _bk);
                                break;
                            }
                        }
                    }
                }

                if (!n.bks.isEmpty()) {
                    Random _random = new Random(1);
                    int[] rands = new int[200];
                    for (int j = 0; j < rands.length; j++) {
                        rands[j] = _random.nextInt(n.bks.size());
                    }

                    for (int j = 0; j < rands.length; j++) {
                        Node ncopy = n.copy();
                        int index = rands[j];
                        Block _bk = n.bks.get(index);
                        for (int k = 0; k < ncopy.sps.size(); k++) {
                            Space _sp = ncopy.sps.get(k);
                            if (_sp.contains(_bk)){
                                ncopy.add_block(_sp, _bk);
                                break;
                            }
                        }
                        TreeSearch ts1 = new TreeSearch(inst);
                        ts1.partial_solve(ncopy, 1, 5, 2);
                        off_sols.add(ts1.bs);
                        ts1.bs.check();
                        System.out.println("The number of off-line solutions: " + off_sols.size());
                    }
                }


                // random load & initialization state
                Solution init_sol1 = new Solution(inst);
                ArrayList<Space> init_sps1 = new ArrayList<>();
                init_sps1.add(new Space(inst));
                Node current = new Node(inst, init_sol1, init_sps1, bks);

                int iter = 0;
                for (int num = 1; num <= pb; num++) { // num represents the number of blocks placed by the worker
                    while (!current.bks.isEmpty() && !current.sps.isEmpty()) {
                        if (iter < num) {
                            int id = placed_id[pb - iter -1 ];
                            Block _bk = get_block(current.bks, id);
                            for (int k = 0; k < current.sps.size(); k++) {
                                Space sp = current.sps.get(k);
                                if (sp.contains(_bk)) {
                                    current.add_block(sp, _bk);
                                    TreeSearch t_s = new TreeSearch(inst);
                                    Solution fs = t_s.partial_solve(current, 1, 5, 2);
                                    fs.check();
                                    double t_s_rate = ((double) fs.vol) / inst.bin.v;
                                    ts_p_rate_sum[iter] += t_s_rate;
                                    System.out.println("Partial tree search loading rate >>> " + t_s_rate);

                                    Solution bsl = Baseline.solve(current);
                                    bsl.check();
                                    double bl_rate = ((double)bsl.vol) / inst.bin.v;
                                    bl_rate_sum[iter] += bl_rate;
                                    System.out.println("Base line loading rate >>> " + bl_rate);

                                    Dynamic dy = new Dynamic(name, i, num, current, bks, off_sols);
                                    Solution dy_sol = dy.get_best_sol();
                                    dy_sol.check();
                                    double dy_rate = ((double)dy_sol.vol) / inst.bin.v;
                                    dy_rate_sum[iter] += dy_rate;
                                    System.out.println("Dynamic recommend solution loading rate >>>" + dy_rate);
                                    break;
                                }
                            }
                            iter++;
                        }
                        else
                            break;
                    }
                }
            }
            double ts_avg = (ts_rate_sum / insts.length) * 100;

            for (int i = 0; i < bl_rate_sum.length; i++) {
                double _ts = ts_p_rate_sum[i] / insts.length * 100;
                double bl = bl_rate_sum[i] / insts.length * 100;
                double dy = dy_rate_sum[i] / insts.length * 100;
                out.write(name + "\t" + (i + 1) + "\t" + String.format("%.2f", ts_avg) + "\t" + String.format("%.2f", _ts) + "\t" + String.format("%.2f", bl) + "\t" + String.format("%.2f", dy));
                out.write('\n');
            }
        }

        out.close();
        System.out.println("result generated.");
    }

    public static Block get_block(ArrayList<Block> bks, int id) {
        for (int i = 0; i < bks.size(); i++) {
            Block _bk = bks.get(i);
            if (_bk.id == id) {
                return _bk;
            }
        }
        return null;
    }
}