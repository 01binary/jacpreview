"use client";

import { useRosListener } from "@/hooks/useRosListener";
import { KinematicsData } from "@/types/KinematicsData";
import { useCallback, useState } from "react";
import styles from "./page.module.scss";
import { initialData } from "./initialData";

export default function Home() {
  const [snapshot, setSnapshot] = useState<KinematicsData>(initialData);

  const handleKinematics = useCallback((data: KinematicsData) => {
    setSnapshot(data);
  }, []);

  useRosListener(handleKinematics);

  return (
    <main>
      <h2>End Effector</h2>
      <table>
        <tr>
          <td>
            <div className={styles.cellLabel}>δ<sub>x</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[0].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>δ<sub>y</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[1].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>δ<sub>z</sub></div>
            <div className={styles.cellValue}>{snapshot?.rot[2].toFixed(2)}</div>
          </td>
        </tr>
        <tr>
          <td>
            <div className={styles.cellLabel}>θx</div>
            <div className={styles.cellValue}>{snapshot?.rot[0].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>θy</div>
            <div className={styles.cellValue}>{snapshot?.rot[1].toFixed(2)}</div>
          </td>
          <td>
            <div className={styles.cellLabel}>θz</div>
            <div className={styles.cellValue}>{snapshot?.rot[2].toFixed(2)}</div>
          </td>
        </tr>
      </table>
      <h2>Joints</h2>
      <table>
        <tr>
          {snapshot?.q_in.map((val, idx) => (
            <td key={idx}>
              <div className={styles.cellLabel}>
                θ<sub>{idx + 1}</sub>
              </div>
              <div className={styles.cellValue}>{val.toFixed(2)}</div>
            </td>
          ))}
        </tr>
      </table>
      <h2>Jacobian</h2>
      <table>
        <tr>
          <td>
            <div className={[styles.cellLabel, styles.callLabelPre].join(' ')}>
              δp<sub>x</sub>
            </div>
            <div className={styles.cellValue}>{0}</div>
          </td>
        </tr>
      </table>
    </main>
  );
}
