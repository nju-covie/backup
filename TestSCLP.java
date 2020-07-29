package packing.Test;

import packing.Data.*;
import packing.Dynamic.Baseline;
import packing.Common.*;
import packing.IO.InstanceLoader2;
import packing.SCLP.*;

import java.io.*;
import java.util.*;

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

            Cuboid container = new Cuboid(589, 235, 239); // Unified container
            Instance[] insts = InstanceLoader2.load_inst(path);
            System.out.println("load instances successfully >>>...");

            double ts_rate_sum = 0;


            for(int i = 0; i < insts.length; i++){
                ArrayList<Solution> off_sols = new ArrayList<>();
                System.out.println("Instance>>>>" + i);
                Instance inst = insts[i];

                inst.buff_size = 8; // set the instance buffer size

                inst.bin = container;
                BlockGeneration bg = new BlockGeneration(inst);
                ArrayList<Block> bks = bg.generate_blocks(100, 100, 100, 10000, 0.99);
                System.out.println("Block Number>>>" + bks.size());
                TreeSearch ts = new TreeSearch(inst);
                System.out.println("start to tree search>>>...");
                Solution s = ts.solve(bks, 1, 25, 2);
                off_sols.add(s);
                double ts_rate = ((double)s.vol) / inst.bin.v;
                ts_rate_sum += ts_rate;
                System.out.println("Tree search loading rate>>> " + ((double)s.vol) / inst.bin.v);
                System.out.println("The number of off-line solutions: " + off_sols.size());

                //create the init state
                Solution init_sol = new Solution(inst);
                ArrayList<Space> init_sps = new ArrayList<>();
                init_sps.add(new Space(inst));
                ArrayList<Block> feas_bks = new ArrayList<>();

                for (int j = 0; j < bks.size(); j++) {
                    Block bk = bks.get(j);
                    int item_num = 0;
                    for (int k = 0; k < bk.type_num.length; k++) {
                        item_num += bk.type_num[k];
                    }
                    if (item_num <= 8) {
                        feas_bks.add(bk);
                    }
                }

                Node n = new Node(inst, init_sol, init_sps, feas_bks);

                Random _random = new Random(1);
                int[] rands = new int[100];
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
                    //ts1.bs.check();
                    System.out.println("The number of off-line solutions: " + off_sols.size());
                }

                // construct the arrival order of item
                Item[] item_seq = new Item[inst.total_num];
                int count = 0;
                for (int j = 0; j < inst.items.length; j++) {
                    Item item = inst.items[j];
                    int num = inst.item_num[j];
                    for (int k = 0; k < num; k++) {
                        item_seq[count] = item;
                        count++;
                    }
                }
                shuffle(item_seq); // shuffle the item array

                double tsrt; // tree search loading rate
                double blrt = 0.0; // base line loading rate
                double drrt = 0.0; // dynamic recommend loading rate

                int[] ts_num = new int[inst.type_num]; // tree search remain item num
                int[] bl_num = new int[inst.type_num]; // base line remain num
                int[] dr_num = new int[inst.type_num]; // dynamic recommend remain num

                int[] tsrm = inst.item_num.clone(); // remaining item in the depot
                int[] blrm = inst.item_num.clone();
                int[] drrm = inst.item_num.clone();

                Item[] tssn = item_seq.clone(); // tree search item arrival sequence
                Item[] blsn = item_seq.clone(); // base line item arrival sequence
                Item[] drsn = item_seq.clone(); // dynamic recommend item arrival sequence

                Instance ts_inst = new Instance(inst.bin, inst.items, new int[inst.type_num], inst.buff_size);
                Instance bl_inst = new Instance(inst.bin, inst.items, new int[inst.type_num], inst.buff_size);
                Instance dr_inst = new Instance(inst.bin, inst.items, new int[inst.type_num], inst.buff_size);

                Node t_s = new Node(ts_inst); // tree search status
                Node b_l = new Node(bl_inst); // base line status
                Node d_r = new Node(dr_inst); // dynamic recommend status

                double t1 = System.nanoTime();

                while ((!t_s.terminate)) {
                    if (tssn[0] != null) {
                        t_s = init_state(ts_inst, inst.item_num, ts_num, tssn);
                    }

                    TreeSearch treeSearch = new TreeSearch(ts_inst);
                    treeSearch.partial(t_s, 1, 25, 2);
                    double load_rate = ((double) t_s.s.vol) / inst.bin.v;
                    System.out.println("Partial tree search loading rate >>> " + load_rate);
                }
                t_s.s.check(inst.item_num);
                tsrt = ((double) t_s.s.vol) / inst.bin.v;
                double t2 = System.nanoTime();
                System.out.println(tsrt);
                System.out.println((t2 - t1) / 1e9);

                while (!b_l.terminate) {
                    if (blsn[0] != null) {
                        b_l = init_state(bl_inst, inst.item_num, bl_num, blsn);
                    }

                    for (int j = 0; j < b_l.sps.size(); j++) {
                        Space sp = b_l.sps.get(j);
                        Block bk = Baseline.best_block(sp, b_l.bks);
                        if (bk != null) {
                            b_l.add_block2(sp, bk);
                            double load_rate = ((double) b_l.s.vol) / inst.bin.v;
                            System.out.println("Base line loading rate >>> " + load_rate);
                            break;
                        }
                    }
                }
                double t3 = System.nanoTime();
                System.out.println((t3 - t2) / 1e9);

                // generate the off-line solution graph
                ArrayList<Graph2> off_graphs = new ArrayList<>();
                for (int j = 0; j < off_sols.size(); j++) {
                    Solution off_sol = off_sols.get(j);
                    Graph2 graph = new Graph2(off_sol.bks, off_sol.pos, null);
                    off_graphs.add(graph);
                }

                // off-line solution set -> off_sols
                while (!d_r.terminate) {
                    if (drsn[0] != null) {
                        d_r = init_state(dr_inst, inst.item_num, dr_num, drsn);
                    }

                    Graph2 cur_gra = new Graph2(d_r.s.bks, d_r.s.pos, null);
                    
                    for (int j = 0; j < off_graphs.size(); j++) {
                        if (off_graphs.get(j).contains(cur_gra)) {

                        }
                    }
                }
                /*Object[] init_buff = buff_area(null, null, item_seq.clone(), buff_size);
                Item[] init_items = (Item[]) init_buff[0];
                int[] init_nums = (int[]) init_buff[1];
                BlockGeneration init_bg = new BlockGeneration(inst);
                ArrayList<Block> init_bks = init_bg.batch_block(100, 100, 100, 10000, 0.99, init_items, init_nums);*/
                /*for (int j = 0; j < off_sols.size(); j++) {
                    Solution off_line = off_sols.get(j);
                    Node dr_copy = d_r.copy();
                    Item[] dr_it_copy = null;
                    int[] dr_num_copy = null;
                    if (dr_it != null) {
                        dr_it_copy = dr_it.clone();
                    }
                    if (dr_num != null) {
                        dr_num_copy = dr_num.clone();
                    }
                    Item[] drsn_copy = drsn.clone();

                    while ((!stock_empty(drsn_copy) || !dr_copy.bks.isEmpty()) && !dr_copy.sps.isEmpty()) {
                        buff_area(dr_num_copy, drsn_copy, buff_size);
                        Item[] items = (Item[]) buff[0];
                        int[] nums = (int[]) buff[1];
                        BlockGeneration bg_bat = new BlockGeneration(inst);
                        ArrayList<Block> bks_bat = bg_bat.batch_block(100, 100, 100, 10000, 0.99, items, nums);
                        dr_copy.bks = new ArrayList<>(bks_bat);

                        if (dr_copy.s.bks.isEmpty()) {
                            for (int k = 0; k < dr_copy.bks.size(); k++) {
                                boolean terminate = false;
                                Block bk1 = dr_copy.bks.get(k);
                                for (int l = 0; l < off_line.bks.size(); l++) {
                                    Block bk2 = off_line.bks.get(l);
                                    if (bk1.match(bk2)) {
                                        dr_copy.add_block(dr_copy.sps.get(0), bk1);
                                        update_remain(items, nums, bk1);
                                        dr_it_copy = items;
                                        dr_num_copy = nums;
                                        double load_rate = ((double) dr_copy.s.vol) / inst.bin.v;
                                        System.out.println("Dynamic loading rate >>> " + load_rate);
                                        terminate = true;
                                        break;
                                    }
                                }
                                if (terminate)
                                    break;
                            }
                        }

                        else {
                            Dynamic2 dy = new Dynamic2(dr_copy);
                            Graph2 off_gra = new Graph2(off_line.bks, off_line.pos, null);
                            Block bk = dy.dynamic(off_gra, dr_copy);
                            update_remain(items, nums, bk);
                            dr_it_copy = items;
                            dr_num_copy = nums;
                            double load_rate = ((double) dr_copy.s.vol) / inst.bin.v;
                            System.out.println("Dynamic loading rate >>> " + load_rate);
                        }

                    }
                }*/
            }
            double ts_avg = (ts_rate_sum / insts.length) * 100;

            /*for (int i = 0; i < bl_rate_sum.length; i++) {
                double _ts = ts_p_rate_sum[i] / insts.length * 100;
                double bl = bl_rate_sum[i] / insts.length * 100;
                double dy = dy_rate_sum[i] / insts.length * 100;
                out.write(name + "\t" + (i + 1) + "\t" + String.format("%.2f", ts_avg) + "\t" + String.format("%.2f", _ts) + "\t" + String.format("%.2f", bl) + "\t" + String.format("%.2f", dy));
                out.write('\n');
            }*/
        }
        out.close();
        System.out.println("result generated.");
    }

    public static Block idToBlock(ArrayList<Block> bks, int id) {
        for (int i = 0; i < bks.size(); i++) {
            Block _bk = bks.get(i);
            if (_bk.id == id) {
                return _bk;
            }
        }
        return null;
    }

    public static Block get_block(ArrayList<Block> bks, Block bk) {
        for (int i = 0; i < bks.size(); i++) {
            Block _bk = bks.get(i);
            if (_bk.hashCode() == bk.hashCode() && _bk.equals(bk)) {
                return _bk;
            }
        }
        return null;
    }

    public static boolean stock_empty(Item[] items) {
        for (int i = 0; i < items.length; i++) {
            if (items[i] != null)
                return false;
        }
        return true;
    }

    /*public static ArrayList<Integer> no_zero(int[] num) {
        ArrayList<Integer> ret = new ArrayList<>();
        for (int i = 0; i < num.length; i++) {
            if (num[i] != 0)
                ret.add(i);
        }
        return ret;
    }*/

    /*public static ArrayList<Block> new_block(Instance inst, Item[] item_type, int[] item_num, Item[] item_seq, int buff_size) {
        Object[] buff = buff_area(item_type, item_num, item_seq, buff_size);
        Item[] items = (Item[]) buff[0];
        int[] nums
    }*/

    public static void swap(Item[] items, int i, int j) {
        Item temp = items[i];
        items[i] = items[j];
        items[j] = temp;
    }

    public static void shuffle(Item[] items) {
        Random rand = new Random(1);
        int length = items.length;
        for (int i = length; i > 0; i--) {
            int randInt = rand.nextInt(i);
            swap(items, randInt, i-1);
        }
    }

    public static Node init_state(Instance inst, int[] total_num, int[] buff_num, Item[] item_seq) {
        Node.buff_area(buff_num, item_seq, inst.buff_size);
        inst.setItem_num(buff_num);
        BlockGeneration bg = new BlockGeneration(inst);
        ArrayList<Block> bks = bg.generate_blocks(100, 100, 100, 10000, 0.99);
        Solution s = new Solution(inst);
        ArrayList<Space> init_sps = new ArrayList<>();
        init_sps.add(new Space(inst));
        Node state = new Node(inst, s, init_sps, bks);
        state.item_seq = item_seq;
        state.depot_num = total_num.clone();
        state.buff_num = buff_num;
        return state;
    }
}
